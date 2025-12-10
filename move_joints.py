import tkinter as tk

root = tk.Tk()
root.title("Robot Joint Control")

def update(section, label, val):
    print(f"{section} - {label}: {val}")

def add_slider(frame, section, label, min_val, max_val):
    tk.Label(frame, text=label).pack()
    tk.Scale(
        frame,
        from_=min_val,
        to=max_val,
        orient='horizontal',
        length=600,
        command=lambda val: update(section, label, val)
    ).pack(pady=3)

sections = {
    "Torso": {
        "Yaw": (-90, 90)
    },
    "Left Leg": {
        "Thigh Roll": (-10, 90),
        "Thigh Pitch": (-90, 90),
        "Thigh Yaw": (-90, 90),
        "Knee Pitch": (0, -130),
        "Ankle Pitch": (-45, 45),
        "Ankle Roll": (-45, 45)
    },
    "Right Leg": {
        "Thigh Roll": (-10, 90),
        "Thigh Pitch": (-90, 90),
        "Thigh Yaw": (-90, 90),
        "Knee Pitch": (0, 130),
        "Ankle Pitch": (-45, 45),
        "Ankle Roll": (-45, 45)
    }
}

for section, joints in sections.items():
    frame = tk.LabelFrame(root, text=section, padx=10, pady=10)
    frame.pack(padx=10, pady=10, fill="x")
    for name, (min_val, max_val) in joints.items():
        add_slider(frame, section, name, min_val, max_val)

root.mainloop()

