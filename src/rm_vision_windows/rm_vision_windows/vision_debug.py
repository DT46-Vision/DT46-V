"""Tk replacement for the DT46 rqt perspective and runtime parameter editor."""

import json
import copy
import os
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, font as tkfont

from PIL import Image, ImageDraw, ImageGrab, ImageTk

from .aim import DEBUG_IMAGES, ROOT
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
        self.images, self.last_parameters = {}, None
        self.theme_config = json.loads((ROOT / "standalone/debug_ui.json").read_text(encoding="utf-8"))
        self.window = tk.Tk()
        self.window.title("DT46 - Windows simulation vision debug (Python)")
        self.style = ttk.Style(self.window)
        self.style.theme_use("clam")  # Native Windows ttk themes ignore custom widget colors.
        self.theme = tk.StringVar(value=self.theme_config["defaults"]["python"])
        self.font_size = tk.IntVar(value=18)
        width = min(1440, self.window.winfo_screenwidth() - 80)
        height = min(900, self.window.winfo_screenheight() - 100)
        self.window.geometry(f"{width}x{height}+20+20")
        if hidden:
            self.window.withdraw()
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        toolbar = ttk.Frame(self.window)
        toolbar.pack(fill="x")
        ttk.Button(toolbar, text="Save frame", command=self.save_frame).pack(side="left")
        ttk.Label(toolbar, text="UI font").pack(side="left", padx=(10, 4))
        self.font_selector = ttk.Combobox(toolbar, textvariable=self.font_size, values=self.theme_config["font_sizes"], width=3, state="readonly")
        self.font_selector.pack(side="left")
        self.font_selector.bind("<<ComboboxSelected>>", lambda _: self.apply_font())
        ttk.Label(toolbar, text="Theme").pack(side="left", padx=(10, 4))
        self.theme_selector = ttk.Combobox(toolbar, textvariable=self.theme, values=("Day", "Night"), width=6, state="readonly")
        self.theme_selector.pack(side="left")
        self.theme_selector.bind("<<ComboboxSelected>>", lambda _: self.apply_theme())
        self.status = tk.StringVar(value="Starting — camera controls are simulated")
        ttk.Label(toolbar, textvariable=self.status).pack(side="left", padx=10)
        body = ttk.Panedwindow(self.window, orient="horizontal")
        body.pack(fill="both", expand=True)
        self.body = body
        body.bind("<Configure>", lambda event: body.sashpos(0, max(200, event.width-int(330*self.font_size.get()/14))))
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
        self.apply_font()
        self.apply_theme()

    def apply_font(self):
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont", "TkFixedFont"):
            tkfont.nametofont(name, root=self.window).configure(size=-self.font_size.get(),
                family="Consolas" if name == "TkFixedFont" else "Segoe UI")
        self.style.configure(".", font="TkDefaultFont")
        self.body.sashpos(0, max(200, self.body.winfo_width()-int(330*self.font_size.get()/14)))
        self.redraw()

    def apply_theme(self):
        p = self.theme_config["themes"][self.theme.get()]
        self.palette = p
        self.window.configure(background=p["window"])
        self.style.configure(".", background=p["window"], foreground=p["text"], bordercolor=p["border"],
                             lightcolor=p["border"], darkcolor=p["border"], fieldbackground=p["edit"],
                             arrowcolor=p["text"], insertcolor=p["edit_cursor"], troughcolor=p["scrollbar"])
        self.style.configure("TButton", background=p["button"])
        self.style.map("TButton", background=[("pressed", p["button_active"]), ("active", p["button_hover"])])
        self.style.map("TCombobox", fieldbackground=[("readonly", p["combo"])], foreground=[("readonly", p["text"])])
        self.style.map("TEntry", fieldbackground=[("readonly", p["header"])])
        self.style.map("TNotebook.Tab", background=[("selected", p["window"]), ("!selected", p["header"])])
        self.style.map("TCheckbutton", background=[("active", p["window"])],
                       indicatorbackground=[("selected", p["toggle_cursor"]), ("!selected", p["toggle"])])
        self.window.option_add("*TCombobox*Listbox.background", p["edit"])
        self.window.option_add("*TCombobox*Listbox.foreground", p["text"])
        self.window.option_add("*TCombobox*Listbox.selectBackground", p["select_active"])
        self.window.option_add("*TCombobox*Listbox.selectForeground", p["text"])
        self.logs.configure(background=p["edit"], foreground=p["text"], insertbackground=p["edit_cursor"])
        for canvas in self.settings_canvases:
            canvas.configure(background=p["window"])
        for _, canvas in self.panes:
            canvas.configure(background=p["image_background"])
        self.redraw()

    def make_settings(self, parent):
        ttk.Label(parent, text="Simulated camera / original vision").pack()
        notebook = ttk.Notebook(parent)
        notebook.pack(fill="both", expand=True)
        self.entries = {}
        self.settings_canvases = []
        groups = {"camera": self.aim.stage.camera_sensors[self.aim.camera].parameters,
                  "detector": {**self.aim.pipeline.settings["detector"], "cls_model_file": self.aim.pipeline.settings["model"]},
                  "tracker": self.aim.pipeline.settings["tracker"]}
        for name, parameters in groups.items():
            page = ttk.Frame(notebook)
            notebook.add(page, text=name)
            canvas = tk.Canvas(page, highlightthickness=0, width=315)
            self.settings_canvases.append(canvas)
            scroll = ttk.Scrollbar(page, orient="vertical", command=canvas.yview)
            canvas.configure(yscrollcommand=scroll.set)
            scroll.pack(side="right", fill="y")
            canvas.pack(side="left", fill="both", expand=True)
            fields = ttk.Frame(canvas)
            canvas.create_window((0, 0), window=fields, anchor="nw")
            fields.bind("<Configure>", lambda _, c=canvas: c.configure(scrollregion=c.bbox("all")))
            self.entries[name] = {}
            for row, (key, value) in enumerate(sorted(parameters.items())):
                ttk.Label(fields, text=key).grid(row=2*row, column=0, sticky="w")
                variable = tk.BooleanVar(value=value) if isinstance(value, bool) else tk.StringVar(value=str(value))
                state = "readonly" if name == "tracker" and key in PHYSICS_INPUTS else "normal"
                if isinstance(value, bool):
                    ttk.Checkbutton(fields, text="Enabled", variable=variable).grid(row=2*row+1, column=0, sticky="w")
                else:
                    ttk.Entry(fields, textvariable=variable, width=28, state=state).grid(row=2*row+1, column=0, sticky="ew")
                self.entries[name][key] = variable
        ttk.Label(parent, text="k / gravity / speed follow shooter config", wraplength=310, justify="left").pack(fill="x")
        ttk.Button(parent, text="Apply (resets tracker)", command=self.apply).pack(fill="x")
        ttk.Button(parent, text="Load parameters JSON", command=self.load_settings).pack(fill="x")
        ttk.Button(parent, text="Save parameters JSON", command=self.save_settings).pack(fill="x")
        self.message = tk.StringVar()
        ttk.Label(parent, textvariable=self.message, wraplength=310).pack(fill="x")
        self.logs = tk.Text(parent, height=6, width=32, state="disabled", font="TkFixedFont")
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
        self.last_parameters = copy.deepcopy(self.aim.parameters())
        self.last_parameters["camera"] = copy.deepcopy(result["sensor"]["parameters"])
        self.images = {name: view_image(name, image, result, self.aim) for name in VIEWS}
        self.status.set(f"t={result['simulation_time_s']:.2f}s  {result['state']} "
                        f"id={result['tracked_id']} fire={int(result['can_fire'])} "
                        f"vision={result['processing_ms']:.1f}ms")
        logs = result["tracker_debug"]["logs"]
        if logs:
            self.logs.configure(state="normal")
            self.logs.insert("end", "".join(f"{result['simulation_time_s']:.6f} {entry['name']}: {json.dumps(entry['value'], separators=(',', ':'))}\n"
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
            frame = self.images.get(choice.get())
            if frame is None:
                canvas.create_text(10, 15, anchor="nw", fill="white", text="No current image / display disabled")
                continue
            display = Image.fromarray(frame).rotate(-options["rotate"].get(), expand=True)
            if not options["zoom"].get():
                scale = min(max(1, canvas.winfo_width())/display.width, max(1, canvas.winfo_height())/display.height)
                display = display.resize((max(1, int(display.width*scale)), max(1, int(display.height*scale))),
                                         Image.Resampling.LANCZOS if options["smooth"].get() else Image.Resampling.NEAREST)
            if options["grid"].get():
                if display.mode != "RGB":
                    display = display.convert("RGB")
                draw = ImageDraw.Draw(display, "RGBA")
                for step in range(1, 4):
                    draw.line((display.width*step//4, 0, display.width*step//4, display.height), fill=(255, 255, 255, 150))
                    draw.line((0, display.height*step//4, display.width, display.height*step//4), fill=(255, 255, 255, 150))
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
            frame = self.images.get(name)
            if frame is not None:
                Image.fromarray(frame).save(folder/f"{name}.png")
        (folder/"result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        (folder/"parameters.json").write_text(json.dumps(self.last_parameters, indent=2), encoding="utf-8")
        self.window.update_idletasks()
        if self.window.winfo_viewable():
            x, y = self.window.winfo_rootx(), self.window.winfo_rooty()
            ImageGrab.grab((x, y, x+self.window.winfo_width(), y+self.window.winfo_height())).save(folder/"window.png")
        else:
            # Hidden test windows have no desktop pixels; label their composed preview explicitly.
            preview = Image.new("RGB", (1000, 760), "#181818")
            draw = ImageDraw.Draw(preview)
            for index, (choice, _) in enumerate(self.panes):
                pixels = self.images.get(choice.get())
                if pixels is not None:
                    tile = Image.fromarray(pixels)
                    tile.thumbnail((490, 350))
                    preview.paste(tile, ((index % 2)*500, (index//2)*380+25))
                draw.text(((index % 2)*500+5, (index//2)*380+5), choice.get(), fill="white")
            preview.save(folder/"window.png")
        (folder/"display.json").write_text(json.dumps({"desktop_capture": bool(self.window.winfo_viewable()),
            "theme": self.theme.get(), "ui_font_px": self.font_size.get(),
            "panes": [{"view": choice.get(), **{key: value.get() for key, value in options.items()}}
                      for (choice, _), options in zip(self.panes, self.view_options)]}, indent=2), encoding="utf-8")
        self.status.set(f"Saved {folder.name}")
        return folder
