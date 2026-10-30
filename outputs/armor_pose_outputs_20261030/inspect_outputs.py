"""Evaluate the frozen CPU model and expose every test prediction for inspection."""
import base64
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch
from PIL import Image, ImageDraw, ImageFont
from torch.utils.data import DataLoader

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.armor_pose_training.data import ArmorCrops, TEAMS, manifest_sha256, read_manifest
from tools.armor_pose_training.evaluate import load_model, metrics
from tools.armor_pose_training.model import OUTPUT_NAMES, rotation_matrix

OUT = Path(__file__).resolve().parent
MANIFEST = ROOT.parent / 'outputs/armor_batch_20261030/dataset_10000/training/training.jsonl'
RUN = ROOT / '.cache/armor-pose-runs/bootstrap_20261030'
NAMES = {'base_small': '基地小板', 'engineer_2': '工程2', 'infantry_3': '步兵3',
         'infantry_4': '步兵4', 'outpost': '前哨站', 'sentry': '哨兵', 'red': '红', 'blue': '蓝'}


def geometry(position: list, rotation: list) -> dict:
    p, r = np.asarray(position), np.asarray(rotation)
    distance = float(np.linalg.norm(p))
    return {'position_m': position, 'normal_camera': r[:, 2].tolist(), 'depth_m': float(p[2]),
            'range_m': distance, 'bearing_yaw_deg': math.degrees(math.atan2(p[0], p[2])),
            'bearing_pitch_deg': math.degrees(math.atan2(-p[1], math.hypot(p[0], p[2]))),
            'incidence_deg': math.degrees(math.acos(float(np.clip(r[:, 2] @ (-p / distance), -1, 1)))),
            'rotation_camera_from_board': rotation}


def collect() -> tuple[list[dict], dict]:
    metadata = json.loads((RUN / 'armor_pose.json').read_text(encoding='utf-8'))
    model_hash = hashlib.sha256((RUN / 'armor_pose.onnx').read_bytes()).hexdigest()
    assert model_hash == metadata['onnx_sha256']
    assert manifest_sha256(MANIFEST) == metadata['training_manifest_sha256']
    data = ArmorCrops(MANIFEST, read_manifest(MANIFEST), 'test', metadata['classes'])
    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    session = ort.InferenceSession(str(RUN / 'armor_pose.onnx'), sess_options=options,
                                   providers=['CPUExecutionProvider'])
    assert [o.name for o in session.get_outputs()] == OUTPUT_NAMES
    reference, state = load_model(RUN / 'best.pt', torch.device('cpu'))
    assert state['epoch'] == metadata['checkpoint_epoch']
    parity_errors = dict.fromkeys(OUTPUT_NAMES, 0.0)
    values, labels = [[] for _ in range(5)], {}
    for batch in DataLoader(data, batch_size=64):
        outputs = session.run(None, {'image': batch['image'].numpy(), 'camera': batch['camera'].numpy()})
        with torch.inference_mode():
            expected = reference(batch['image'], batch['camera'])
        for name, actual, wanted in zip(OUTPUT_NAMES, outputs, expected):
            np.testing.assert_allclose(actual, wanted.numpy(), atol=1e-4, rtol=1e-4, err_msg=name)
            parity_errors[name] = max(parity_errors[name], float(np.abs(actual - wanted.numpy()).max()))
        for dest, output in zip(values, outputs):
            dest.append(torch.from_numpy(output))
        for key, value in batch.items():
            if key not in {'image', 'camera'}:
                labels.setdefault(key, []).append(value)
    outputs = tuple(torch.cat(v) for v in values)
    target = {k: torch.cat(v) for k, v in labels.items()}
    result = metrics(outputs, target)
    predicted_r, valid_r = rotation_matrix(outputs[4])
    true_r, _ = rotation_matrix(target['rotation'])
    records = []
    for i, row in enumerate(data.rows):
        sticker = metadata['classes'][int(outputs[1][i].argmax())]
        team = TEAMS[int(outputs[2][i].argmax())]
        probability = float(outputs[0][i].sigmoid())
        truth = row['target']
        pred = {'is_armor': probability >= .5, 'armor_probability': probability,
                'sticker': sticker, 'team': team, 'rotation_6d': outputs[4][i].tolist(),
                'geometry': geometry(outputs[3][i].tolist(), predicted_r[i].tolist()) if valid_r[i] else None}
        actual = {'is_armor': row['is_armor'], 'sticker': truth['sticker_type'] if truth else None,
                  'team': truth['team'] if truth else None,
                  'geometry': geometry(target['position'][i].tolist(), true_r[i].tolist()) if truth else None}
        flags = {'presence': pred['is_armor'] == actual['is_armor']}
        for key in ('sticker', 'team'):
            flags[key] = pred[key] == actual[key] if row['supervision'][key] else None
        errors = {}
        if row['supervision']['pose'] and pred['geometry'] is not None:
            for name in ('depth_m', 'range_m', 'bearing_yaw_deg', 'bearing_pitch_deg', 'incidence_deg'):
                errors[name] = pred['geometry'][name] - actual['geometry'][name]
            cosine = (predicted_r[i] * true_r[i]).sum().item()
            errors['rotation_deg'] = math.degrees(math.acos(float(np.clip((cosine - 1) / 2, -1, 1))))
        image = (MANIFEST.parent / row['image']).resolve()
        records.append({'sample_id': row['sample_id'], 'scene_group': row['scene_group'],
                        'image': str(image), 'supervision': row['supervision'],
                        'predicted': pred, 'truth': actual, 'matches': flags, 'errors': errors})
    result.update(checkpoint_epoch=metadata['checkpoint_epoch'], backend='ONNX Runtime CPU',
                  onnx_sha256=model_hash, manifest_sha256=manifest_sha256(MANIFEST), split='test')
    result['cpu_export_parity'] = {'crops': len(data), 'batch_size': 64, 'atol': 1e-4, 'rtol': 1e-4,
                                  'maximum_absolute_error': parity_errors, 'passed': True}
    result['output_origin'] = {
        'network': OUTPUT_NAMES,
        'derived': ['depth_m = position_camera_m[2]', 'range_m = norm(position_camera_m)',
                    'bearing_yaw/pitch = angles of position_camera_m',
                    'incidence = angle between board normal and direction to camera'],
        'independent_depth_range_angle_heads': False,
    }
    return records, result


def add_statistics(records: list[dict], result: dict) -> None:
    result['classification_counts'] = {key: {'correct': sum(r['matches'][key] is True for r in records),
                                           'scored': sum(r['matches'][key] is not None for r in records)}
                                       for key in ('presence', 'sticker', 'team')}
    eligible = [r for r in records if r['truth']['is_armor'] and r['matches']['team'] is not None]
    result['joint_recognition'] = {'correct': sum(all(r['matches'].values()) for r in eligible),
                                  'scored': len(eligible), 'fields': ['presence', 'sticker', 'team']}
    result['per_sticker'] = {}
    result['sticker_confusion'] = {}
    for name in NAMES:
        group = [r for r in records if r['truth']['sticker'] == name and r['matches']['sticker'] is not None]
        if not group:
            continue
        result['per_sticker'][name] = {'count': len(group), 'correct': sum(r['matches']['sticker'] for r in group)}
        result['sticker_confusion'][name] = {pred: sum(r['predicted']['sticker'] == pred for r in group)
                                           for pred in NAMES if pred not in TEAMS}
    result['derived_errors'] = {}
    for name in ('depth_m', 'range_m', 'bearing_yaw_deg', 'bearing_pitch_deg', 'incidence_deg', 'rotation_deg'):
        error = np.array([r['errors'][name] for r in records if name in r['errors']])
        result['derived_errors'][name] = {'count': len(error), 'mean_signed': float(error.mean()),
                                         'absolute_mean': float(np.abs(error).mean()),
                                         'absolute_p50': float(np.quantile(np.abs(error), .5)),
                                         'absolute_p95': float(np.quantile(np.abs(error), .95))}
    old = json.loads((RUN / 'test_metrics.json').read_text())
    for key in ('tp', 'fp', 'fn', 'tn'):
        assert result['presence'][key] == old['presence'][key]
    for key in ('sticker', 'team'):
        assert abs(result[key]['accuracy'] - old[key]['accuracy']) < 1e-7
    result['previous_gpu_comparison'] = {
        'classification_counts_match': True,
        'depth_p50_delta_m': result['pose_all_supervised']['depth_absolute_p50_m'] - old['pose_all_supervised']['depth_absolute_p50_m'],
        'depth_p95_delta_m': result['pose_all_supervised']['depth_absolute_p95_m'] - old['pose_all_supervised']['depth_absolute_p95_m'],
        'note': 'CPU measurements are reported directly; GPU summary is historical, not a per-output export parity reference.',
    }


def select_examples(records: list[dict]) -> list[dict]:
    selected = []
    for name in NAMES:
        group = sorted([r for r in records if r['truth']['sticker'] == name], key=lambda r: r['truth']['geometry']['depth_m'])
        if group:
            selected.append({'reason': '该类别距离居中样本', 'case': group[len(group) // 2]})
    criteria = [('贴纸识别错误', lambda r: r['matches']['sticker'] is False),
                ('阵营识别错误', lambda r: r['matches']['team'] is False),
                ('负样本误检', lambda r: not r['truth']['is_armor'] and r['predicted']['is_armor']),
                ('真实装甲漏检', lambda r: r['truth']['is_armor'] and not r['predicted']['is_armor'])]
    for reason, predicate in criteria:
        found = next((r for r in records if predicate(r) and not any(s['case'] is r for s in selected)), None)
        if found:
            selected.append({'reason': reason, 'case': found})
    for key, reason in [('depth_m', '深度误差最大样本'), ('rotation_deg', '姿态误差最大样本')]:
        selected.append({'reason': reason, 'case': max((r for r in records if key in r['errors']), key=lambda r: abs(r['errors'][key]))})
    return selected


def preview(selected: list[dict]) -> None:
    font_path = 'C:/Windows/Fonts/msyh.ttc'
    regular, small = ImageFont.truetype(font_path, 18), ImageFont.truetype(font_path, 16)
    title = ImageFont.truetype('C:/Windows/Fonts/msyhbd.ttc', 28)
    canvas = Image.new('RGB', (1536, 104 + math.ceil(len(selected) / 3) * 294), '#eff3f8')
    draw = ImageDraw.Draw(canvas)
    draw.text((24, 18), '测试集逐项对照 · 固定第 18 轮 CPU 模型', fill='#152536', font=title)
    draw.text((24, 62), '真=仿真真值，预=预测位姿及其派生量。深度取 Z，偏角由位姿计算；类别居中样本 + 明示失败样本。', fill='#485666', font=regular)
    for i, entry in enumerate(selected):
        row, col = divmod(i, 3)
        x, y = 16 + col * 512, 104 + row * 294
        r = entry['case']; p, t = r['predicted'], r['truth']
        draw.rounded_rectangle((x, y, x + 496, y + 280), radius=12, fill='white')
        draw.text((x + 12, y + 10), entry['reason'], fill='#172a42', font=regular)
        draw.text((x + 12, y + 38), r['sample_id'], fill='#64738a', font=small)
        with Image.open(r['image']) as image:
            canvas.paste(image.convert('RGB'), (x + 12, y + 70))
        truth_text = f"{NAMES[t['sticker']]} / {NAMES[t['team']]}" if t['is_armor'] else '非装甲'
        pred_text = f"{NAMES[p['sticker']]} / {NAMES[p['team']]}" if p['is_armor'] else '非装甲'
        correct = r['matches']['presence'] and r['matches']['sticker'] is not False and r['matches']['team'] is not False
        draw.text((x + 154, y + 70), '真：' + truth_text, fill='#334155', font=regular)
        draw.text((x + 154, y + 100), '预：' + pred_text, fill='#137553' if correct else '#be3d36', font=regular)
        draw.text((x + 154, y + 130), f"装甲分数 {p['armor_probability']:.3f}", fill='#64738a', font=small)
        if t['geometry']:
            a, b = t['geometry'], p['geometry']
            draw.text((x + 154, y + 160), f"深度 真 {a['depth_m']:.2f} / 预 {b['depth_m']:.2f} m", fill='#334155', font=small)
            draw.text((x + 154, y + 188), f"姿态误差 {r['errors']['rotation_deg']:.1f}°", fill='#805311', font=small)
            draw.text((x + 12, y + 220), f"左右偏角 真 {a['bearing_yaw_deg']:+.2f}° / 预 {b['bearing_yaw_deg']:+.2f}°", fill='#334155', font=small)
            draw.text((x + 12, y + 246), f"板面斜视 真 {a['incidence_deg']:.1f}° / 预 {b['incidence_deg']:.1f}°", fill='#334155', font=small)
        else:
            draw.text((x + 154, y + 164), '无位姿真值，不评分', fill='#64738a', font=small)
    canvas.save(OUT / 'preview.png')


def write_review(records: list[dict]) -> None:
    rows = []
    for r in records:
        rows.append({**r, 'image_data': base64.b64encode(Path(r['image']).read_bytes()).decode('ascii')})
    template = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>测试集输出对照</title>
<style>body{font:15px system-ui;background:#f2f5f9;color:#182b40;margin:28px}h1{font-size:25px}select,button{padding:9px;margin:8px 5px 16px 0}table{width:100%;background:white;border-collapse:collapse}td,th{padding:12px;text-align:left;border-bottom:1px solid #dce3ea}td img{width:96px;height:96px}small{color:#64748b}.bad{color:#bd3434}.ok{color:#137553}pre{white-space:pre-wrap;font-size:12px;max-width:650px}details{max-width:650px}</style>
<h1>1,122 张独立测试裁图 · 输出与真值逐项对照</h1><p>固定第 18 轮权重，CPU ONNX。贴纸/阵营列显示原始分类头结果；“是否装甲”判为否时，其他头不应作为有效识别采纳。无真值或未开放监督的项不评分。角度单位 °，位置和深度单位 m。</p>
<p>网络只预测是否装甲、贴纸、阵营和位姿。深度取位置的 Z；距离、左右/上下偏角、板面斜视角均由这份位姿计算，没有额外的深度或角度头。</p>
<select id="filter"><option value="all">全部样本</option><option value="sticker">贴纸错</option><option value="team">阵营错</option><option value="presence">误检或漏检</option></select><select id="order"><option value="original">原始顺序</option><option value="depth_m">深度误差从大到小</option><option value="rotation_deg">姿态误差从大到小</option></select><button id="prev">上一页</button><button id="next">下一页</button><span id="count"></span>
<table><thead><tr><th>测试图</th><th>是否装甲<br>真 → 预</th><th>贴纸 / 阵营<br>真 → 预</th><th>位置与派生距离<br>真 → 预</th><th>朝向与派生偏角</th><th>完整对照记录</th></tr></thead><tbody id="body"></tbody></table>
<script>const data=__DATA__;const names=__NAMES__;let page=0;
const [filter,order,count,body,prev,next]=['filter','order','count','body','prev','next'].map(id=>document.getElementById(id));
const e=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const f=n=>Number(n).toFixed(3);const name=n=>names[n]||'—';
function show(){let a=data.filter(r=>filter.value==='all'||r.matches[filter.value]===false);const score=r=>r.errors[order.value]===undefined?-Infinity:Math.abs(r.errors[order.value]);if(order.value!=='original')a.sort((x,y)=>score(y)-score(x));page=Math.min(page,Math.max(0,Math.ceil(a.length/16)-1));count.textContent=`共 ${a.length} 张，第 ${page+1} / ${Math.max(1,Math.ceil(a.length/16))} 页`;body.innerHTML=a.slice(page*16,page*16+16).map(r=>{const p=r.predicted,t=r.truth,pg=p.geometry,tg=t.geometry;const raw={...r};delete raw.image_data;return `<tr><td><img src="data:image/png;base64,${r.image_data}"><br><small>${e(r.sample_id)}</small></td><td class="${r.matches.presence?'ok':'bad'}">${t.is_armor?'是':'否'} → ${p.is_armor?'是':'否'}<br><small>分数 ${f(p.armor_probability)}</small></td><td><span class="${r.matches.sticker===false?'bad':''}">${name(t.sticker)} → ${name(p.sticker)}</span><br><span class="${r.matches.team===false?'bad':''}">${name(t.team)} → ${name(p.team)}</span>${r.supervision.team?'':'<br><small>阵营无监督，不评分</small>'}</td><td>${tg&&pg?'位置 ['+tg.position_m.map(f).join(', ')+']<br>→ ['+pg.position_m.map(f).join(', ')+']<br>深度 Z '+f(tg.depth_m)+' → '+f(pg.depth_m)+'<br>直线距离 '+f(tg.range_m)+' → '+f(pg.range_m):'—'}</td><td>${tg&&pg?'旋转误差 '+f(r.errors.rotation_deg)+'°<br>左右偏角 '+f(tg.bearing_yaw_deg)+' → '+f(pg.bearing_yaw_deg)+'<br>上下偏角 '+f(tg.bearing_pitch_deg)+' → '+f(pg.bearing_pitch_deg)+'<br>斜视角 '+f(tg.incidence_deg)+' → '+f(pg.incidence_deg):'无位姿真值'}</td><td><details><summary>位置、旋转、监督与误差</summary><pre>${e(JSON.stringify(raw,null,2))}</pre></details></td></tr>`}).join('');}
filter.onchange=order.onchange=()=>{page=0;show()};prev.onclick=()=>{page=Math.max(0,page-1);show()};next.onclick=()=>{page++;show()};show();</script></html>'''
    text = template.replace('__DATA__', json.dumps(rows, ensure_ascii=False).replace('<', '\\u003c'))
    text = text.replace('__NAMES__', json.dumps(NAMES, ensure_ascii=False))
    (OUT / 'review.html').write_text(text, encoding='utf-8')


def main() -> None:
    torch.set_num_threads(4)
    records, result = collect()
    add_statistics(records, result)
    selected = select_examples(records)
    (OUT / 'predictions.jsonl').write_text(''.join(json.dumps(r, ensure_ascii=False, allow_nan=False) + '\n' for r in records), encoding='utf-8')
    (OUT / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    (OUT / 'selected_cases.json').write_text(json.dumps(selected, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    preview(selected)
    write_review(records)
    print(json.dumps({k: result[k] for k in ('classification_counts', 'joint_recognition', 'per_sticker', 'derived_errors')}, ensure_ascii=True))


if __name__ == '__main__':
    main()
