"""Tk replacement for the DT46 rqt perspective and runtime parameter editor."""

import json
import os
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog

from PIL import Image, ImageDraw, ImageGrab, ImageTk

from .aim import DEBUG_IMAGES
from .debug import ballistic_image, tracking_image


VIEWS = ("image_raw", "ballistic_img", "tracking_state_img", *DEBUG_IMAGES)
PHYSICS_INPUTS = ("k_v2", "gravity", "bullet_speed")


def view_image(name, image, result, aim):
    if name == "image_raw":
        return image
    if name == "ballistic_img":
        return ballistic_image(image, result, aim.pipeline.settings["tracker"]["text_size"]) if aim.pipeline.settings["tracker"]["display"] else None
    if name == "tracking_state_img":
        return tracking_image(image, result, aim.pipeline.settings["tracker"]["text_size"]) if aim.pipeline.settings["tracker"]["display"] else None
    return aim.pipeline.debug_image(name)


class VisionDebug:
    def __init__(self, aim, output: Path, hidden: bool = False):
        if os.name == "nt":
            import ctypes
            ctypes.windll.user32.SetProcessDPIAware()
        self.aim, self.output = aim, output
        self.running = True
        self.last_frame = None
        self.window = tk.Tk()
        self.window.title("DT46 — Native vision debug / rqt migration")
        width = min(1440, self.window.winfo_screenwidth() - 80)
        height = min(900, self.window.winfo_screenheight() - 100)
        self.window.geometry(f"{width}x{height}+20+20")
        if hidden:
            self.window.withdraw()
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        toolbar = ttk.Frame(self.window)
        toolbar.pack(fill="x")
        ttk.Button(toolbar, text="Save frame", command=self.save_frame).pack(side="left")
        self.status = tk.StringVar(value="Starting — camera controls are simulated")
        ttk.Label(toolbar, textvariable=self.status).pack(side="left", padx=10)
        body = ttk.Panedwindow(self.window, orient="horizontal")
        body.pack(fill="both", expand=True)
        images, settings = ttk.Frame(body), ttk.Frame(body, width=330)
        body.add(images, weight=4)
        body.add(settings, weight=1)
        self.panes = []
        self.view_options = []
        for index, default in enumerate(("result", "ballistic_img", "img_armor_processed", "bin_img")):
            frame = ttk.Frame(images)
            frame.grid(row=index//2, column=index%2, sticky="nsew")
            choice = tk.StringVar(value=default)
            combo = ttk.Combobox(frame, textvariable=choice, values=VIEWS, state="readonly")
            combo.pack(fill="x")
            combo.bind("<<ComboboxSelected>>", lambda _: self.redraw())
            options = {"smooth": tk.BooleanVar(value=False), "zoom": tk.BooleanVar(value=False),
                       "grid": tk.BooleanVar(value=False), "rotate": tk.IntVar(value=0)}
            controls = ttk.Frame(frame)
            controls.pack(fill="x")
            for key, title in (("smooth", "Smooth"), ("zoom", "1:1"), ("grid", "Grid")):
                ttk.Checkbutton(controls, text=title, variable=options[key], command=self.redraw).pack(side="left")
            rotate = ttk.Combobox(controls, textvariable=options["rotate"], values=(0, 90, 180, 270), width=4, state="readonly")
            rotate.pack(side="left")
            rotate.bind("<<ComboboxSelected>>", lambda _: self.redraw())
            canvas = tk.Canvas(frame, background="#181818", highlightthickness=0)
            xscroll = ttk.Scrollbar(frame, orient="horizontal", command=canvas.xview)
            yscroll = ttk.Scrollbar(frame, orient="vertical", command=canvas.yview)
            canvas.configure(xscrollcommand=xscroll.set, yscrollcommand=yscroll.set)
            xscroll.pack(side="bottom", fill="x")
            yscroll.pack(side="right", fill="y")
            canvas.pack(fill="both", expand=True)
            self.panes.append((choice, canvas))
            self.view_options.append(options)
        for index in range(2):
            images.rowconfigure(index, weight=1, uniform="views")
            images.columnconfigure(index, weight=1, uniform="views")
        self.make_settings(settings)

    def make_settings(self, parent):
        ttk.Label(parent, text="Simulated camera / original detector & tracker").pack()
        notebook = ttk.Notebook(parent)
        notebook.pack(fill="both", expand=True)
        self.entries = {}
        groups = {"camera": self.aim.stage.camera_sensors[self.aim.camera].parameters,
                  "detector": {**self.aim.pipeline.settings["detector"], "cls_model_file": self.aim.pipeline.settings["model"]},
                  "tracker": self.aim.pipeline.settings["tracker"]}
        for name, parameters in groups.items():
            page = ttk.Frame(notebook)
            notebook.add(page, text=name)
            canvas = tk.Canvas(page, highlightthickness=0, width=315)
            scroll = ttk.Scrollbar(page, orient="vertical", command=canvas.yview)
            canvas.configure(yscrollcommand=scroll.set)
            scroll.pack(side="right", fill="y")
            canvas.pack(side="left", fill="both", expand=True)
            fields = ttk.Frame(canvas)
            canvas.create_window((0, 0), window=fields, anchor="nw")
            fields.bind("<Configure>", lambda _, c=canvas: c.configure(scrollregion=c.bbox("all")))
            self.entries[name] = {}
            for row, (key, value) in enumerate(parameters.items()):
                ttk.Label(fields, text=key).grid(row=row, column=0, sticky="w")
                variable = tk.BooleanVar(value=value) if isinstance(value, bool) else tk.StringVar(value=str(value))
                state = "readonly" if name == "tracker" and key in PHYSICS_INPUTS else "normal"
                if isinstance(value, bool):
                    ttk.Checkbutton(fields, variable=variable).grid(row=row, column=1)
                else:
                    ttk.Entry(fields, textvariable=variable, width=11, state=state).grid(row=row, column=1)
                self.entries[name][key] = variable
        ttk.Label(parent, text="k / gravity / initial speed follow shooter config").pack()
        ttk.Button(parent, text="Apply (resets tracker)", command=self.apply).pack(fill="x")
        ttk.Button(parent, text="Load parameters JSON", command=self.load_settings).pack(fill="x")
        ttk.Button(parent, text="Save parameters JSON", command=self.save_settings).pack(fill="x")
        self.message = tk.StringVar()
        ttk.Label(parent, textvariable=self.message, wraplength=310).pack(fill="x")
        self.logs = tk.Text(parent, height=10, width=38, state="disabled")
        self.logs.pack(fill="x")

    def close(self):
        self.running = False

    def pump(self):
        self.window.update_idletasks()
        self.window.update()

    def settings(self):
        settings = {name: {key: (value.get() if isinstance(value, tk.BooleanVar) or key == "cls_model_file" else float(value.get()))
                           for key, value in fields.items()} for name, fields in self.entries.items()}
        settings["model"] = settings["detector"].pop("cls_model_file")
        return settings

    def refresh_parameters(self):
        settings = self.aim.parameters()
        for group, fields in self.entries.items():
            for key, variable in fields.items():
                variable.set(settings["model"] if key == "cls_model_file" else settings[group][key])

    def apply(self):
        try:
            settings = self.settings()
            self.aim.apply_parameters(settings)
            self.last_frame = None
            self.status.set("Applied — tracker reset; YAML unchanged")
            self.message.set(self.status.get())
        except (ValueError, RuntimeError, OSError) as error:
            self.status.set(str(error))
            self.message.set(str(error))

    def load_settings(self):
        filename = filedialog.askopenfilename(filetypes=[("Vision settings", "*.json")])
        if filename:
            try:
                settings = json.loads(Path(filename).read_text(encoding="utf-8"))
                if not isinstance(settings, dict):
                    raise ValueError("Vision parameter JSON must be an object")
                if "flip_image" in settings.get("camera", {}):
                    raise ValueError("Remove camera.flip_image; use display rotation")
                for group, fields in self.entries.items():
                    for key, variable in fields.items():
                        if key in settings.get(group, {}) and not (group == "tracker" and key in PHYSICS_INPUTS):
                            variable.set(settings[group][key])
                if "model" in settings:
                    self.entries["detector"]["cls_model_file"].set(settings["model"])
                self.apply()
            except (ValueError, OSError, TypeError) as error:
                self.status.set(str(error))

    def save_settings(self):
        filename = filedialog.asksaveasfilename(defaultextension=".json", initialfile="vision_parameters.json")
        if filename:
            try:
                Path(filename).write_text(json.dumps(self.settings(), indent=2), encoding="utf-8")
                self.status.set(f"Saved {filename}")
            except (ValueError, OSError) as error:
                self.status.set(str(error))

    def update(self, image, result):
        self.last_frame = image, result
        self.status.set(f"t={result['simulation_time_s']:.2f}s  {result['state']}  "
                        f"id={result['tracked_id']}  fire={int(result['can_fire'])}  "
                        f"vision={result['processing_ms']:.1f}ms")
        logs = result["tracker_debug"]["logs"]
        if logs:
            self.logs.configure(state="normal")
            self.logs.insert("end", "".join(f"{result['simulation_time_s']:.3f} {entry['name']}: {entry['value']}\n"
                                           for entry in logs))
            count = int(self.logs.index("end-1c").split(".")[0])
            if count > 1000:
                self.logs.delete("1.0", f"{count-1000}.0")
            self.logs.see("end")
            self.logs.configure(state="disabled")
        self.redraw()

    def redraw(self):
        if self.last_frame is None:
            return
        image, result = self.last_frame
        for (choice, canvas), options in zip(self.panes, self.view_options):
            canvas.delete("all")
            frame = view_image(choice.get(), image, result, self.aim)
            if frame is None:
                canvas.create_text(10, 15, anchor="nw", fill="white", text="No current image")
                continue
            display = Image.fromarray(frame).rotate(options["rotate"].get(), expand=True)
            if not options["zoom"].get():
                display.thumbnail((max(100, canvas.winfo_width()), max(100, canvas.winfo_height())),
                                  Image.Resampling.LANCZOS if options["smooth"].get() else Image.Resampling.NEAREST)
            if options["grid"].get():
                draw = ImageDraw.Draw(display)
                for step in range(1, 4):
                    draw.line((display.width*step//4, 0, display.width*step//4, display.height), fill="yellow")
                    draw.line((0, display.height*step//4, display.width, display.height*step//4), fill="yellow")
            canvas.frame = ImageTk.PhotoImage(display)
            canvas.create_image(0, 0, anchor="nw", image=canvas.frame)
            canvas.configure(scrollregion=(0, 0, display.width, display.height))

    def save_frame(self):
        if self.last_frame is None:
            return
        image, result = self.last_frame
        folder = self.output/f"g{result['reset_generation']}_p{result['parameter_revision']}_f{result['frame_number']}"
        folder.mkdir(parents=True, exist_ok=True)
        for name in VIEWS:
            frame = view_image(name, image, result, self.aim)
            if frame is not None:
                Image.fromarray(frame).save(folder/f"{name}.png")
        (folder/"result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        (folder/"parameters.json").write_text(json.dumps(self.aim.parameters(), indent=2), encoding="utf-8")
        self.window.update_idletasks()
        if self.window.winfo_viewable():
            x, y = self.window.winfo_rootx(), self.window.winfo_rooty()
            ImageGrab.grab((x, y, x+self.window.winfo_width(), y+self.window.winfo_height())).save(folder/"window.png")
        else:
            # Hidden test windows have no desktop pixels; label their composed preview explicitly.
            preview = Image.new("RGB", (1000, 760), "#181818")
            draw = ImageDraw.Draw(preview)
            for index, (choice, _) in enumerate(self.panes):
                pixels = view_image(choice.get(), image, result, self.aim)
                if pixels is not None:
                    tile = Image.fromarray(pixels)
                    tile.thumbnail((490, 350))
                    preview.paste(tile, ((index % 2)*500, (index//2)*380+25))
                draw.text(((index % 2)*500+5, (index//2)*380+5), choice.get(), fill="white")
            preview.save(folder/"window.png")
        (folder/"display.json").write_text(json.dumps({"desktop_capture": bool(self.window.winfo_viewable()),
            "panes": [{"view": choice.get(), **{key: value.get() for key, value in options.items()}}
                      for (choice, _), options in zip(self.panes, self.view_options)]}, indent=2), encoding="utf-8")
        self.status.set(f"Saved {folder.name}")
        return folder
