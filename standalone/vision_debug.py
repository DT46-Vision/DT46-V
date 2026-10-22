"""Tk replacement for the DT46 rqt perspective and runtime parameter editor."""

import json
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog

from PIL import Image, ImageTk

from .aim import DEBUG_IMAGES
from .debug import ballistic_image, tracking_image


VIEWS = ("image_raw", "ballistic_img", "tracking_state_img", *DEBUG_IMAGES)
PHYSICS_INPUTS = ("k_v2", "gravity", "bullet_speed")


def view_image(name, image, result, aim):
    if name == "image_raw":
        return image
    if name == "ballistic_img":
        return ballistic_image(image, result)
    if name == "tracking_state_img":
        return tracking_image(image, result)
    return aim.pipeline.debug_image(name)


class VisionDebug:
    def __init__(self, aim, output: Path):
        self.aim, self.output = aim, output
        self.running = True
        self.last_frame = None
        self.window = tk.Tk()
        self.window.title("DT46 — Native vision debug / rqt migration")
        self.window.geometry("1440x900")
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
        for index, default in enumerate(("image_raw", "ballistic_img", "img_armor", "img_armor_processed")):
            frame = ttk.Frame(images)
            frame.grid(row=index//2, column=index%2, sticky="nsew")
            choice = tk.StringVar(value=default)
            combo = ttk.Combobox(frame, textvariable=choice, values=VIEWS, state="readonly")
            combo.pack(fill="x")
            combo.bind("<<ComboboxSelected>>", lambda _: self.redraw())
            label = tk.Label(frame, background="#181818", foreground="white")
            label.pack(fill="both", expand=True)
            self.panes.append((choice, label))
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
                  "detector": self.aim.pipeline.settings["detector"],
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
                variable = tk.StringVar(value=str(int(value)) if isinstance(value, bool) else str(value))
                state = "readonly" if name == "tracker" and key in PHYSICS_INPUTS else "normal"
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
        return {name: {key: float(value.get()) for key, value in fields.items()}
                for name, fields in self.entries.items()}

    def apply(self):
        try:
            settings = self.settings()
            # Validate camera without mutating the running sensor on failure.
            sensor = self.aim.stage.camera_sensors[self.aim.camera]
            sensor.validate(**settings["camera"])
            tracker = {k: v for k, v in settings["tracker"].items() if k not in PHYSICS_INPUTS}
            self.aim.pipeline.configure(detector=settings["detector"], tracker=tracker)
            sensor.configure(**settings["camera"])
            self.aim.last_time = None
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
                for group, fields in self.entries.items():
                    for key, variable in fields.items():
                        if key in settings.get(group, {}) and not (group == "tracker" and key in PHYSICS_INPUTS):
                            variable.set(str(settings[group][key]))
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
        for choice, label in self.panes:
            frame = view_image(choice.get(), image, result, self.aim)
            if frame is None:
                label.configure(image="", text="No current armor crop")
                continue
            display = Image.fromarray(frame)
            display.thumbnail((max(100, label.winfo_width()), max(100, label.winfo_height())))
            label.frame = ImageTk.PhotoImage(display)
            label.configure(image=label.frame, text="")

    def save_frame(self):
        if self.last_frame is None:
            return
        image, result = self.last_frame
        folder = self.output/f"frame_{result['simulation_time_s']:.6f}"
        folder.mkdir(parents=True, exist_ok=True)
        for name in VIEWS:
            frame = view_image(name, image, result, self.aim)
            if frame is not None:
                Image.fromarray(frame).save(folder/f"{name}.png")
        (folder/"result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        (folder/"parameters.json").write_text(json.dumps(self.settings(), indent=2), encoding="utf-8")
        self.status.set(f"Saved {folder.name}")
