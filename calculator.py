"""
Simple Calculator
==================
Tkinter 기반 사칙연산 계산기
"""

import tkinter as tk

COLORS = {
    "bg": "#1e1e2e",
    "display_bg": "#16213e",
    "text": "#ffffff",
    "btn_num": "#0f3460",
    "btn_op": "#e94560",
    "btn_eq": "#2ecc71",
    "btn_clear": "#e74c3c",
}


class Calculator(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("계산기")
        self.resizable(False, False)
        self.configure(bg=COLORS["bg"])

        self.expression = ""
        self._build_display()
        self._build_buttons()

    def _build_display(self):
        self.display_var = tk.StringVar(value="0")
        display = tk.Entry(
            self,
            textvariable=self.display_var,
            font=("Consolas", 28),
            bg=COLORS["display_bg"],
            fg=COLORS["text"],
            bd=0,
            justify="right",
            insertwidth=0,
            state="readonly",
            readonlybackground=COLORS["display_bg"],
        )
        display.grid(row=0, column=0, columnspan=4, sticky="nsew", padx=10, pady=10, ipady=20)

    def _build_buttons(self):
        buttons = [
            ("C", 1, 0, "clear"), ("(", 1, 1, "op"), (")", 1, 2, "op"), ("/", 1, 3, "op"),
            ("7", 2, 0, "num"), ("8", 2, 1, "num"), ("9", 2, 2, "num"), ("*", 2, 3, "op"),
            ("4", 3, 0, "num"), ("5", 3, 1, "num"), ("6", 3, 2, "num"), ("-", 3, 3, "op"),
            ("1", 4, 0, "num"), ("2", 4, 1, "num"), ("3", 4, 2, "num"), ("+", 4, 3, "op"),
            ("0", 5, 0, "num"), (".", 5, 1, "num"), ("⌫", 5, 2, "clear"), ("=", 5, 3, "eq"),
        ]

        style_map = {
            "num": COLORS["btn_num"],
            "op": COLORS["btn_op"],
            "eq": COLORS["btn_eq"],
            "clear": COLORS["btn_clear"],
        }

        for (label, row, col, kind) in buttons:
            btn = tk.Button(
                self,
                text=label,
                font=("Consolas", 18, "bold"),
                bg=style_map[kind],
                fg=COLORS["text"],
                bd=0,
                activebackground=style_map[kind],
                activeforeground=COLORS["text"],
                command=lambda l=label: self._on_press(l),
            )
            btn.grid(row=row, column=col, sticky="nsew", padx=4, pady=4, ipadx=10, ipady=14)

        for i in range(6):
            self.grid_rowconfigure(i, weight=1)
        for i in range(4):
            self.grid_columnconfigure(i, weight=1)

        self.bind("<Key>", self._on_key)

    def _on_press(self, label):
        if label == "C":
            self.expression = ""
        elif label == "⌫":
            self.expression = self.expression[:-1]
        elif label == "=":
            self._evaluate()
            return
        else:
            self.expression += label

        self.display_var.set(self.expression if self.expression else "0")

    def _on_key(self, event):
        char = event.char
        if char in "0123456789+-*/().":
            self.expression += char
            self.display_var.set(self.expression)
        elif event.keysym == "Return":
            self._evaluate()
        elif event.keysym == "BackSpace":
            self.expression = self.expression[:-1]
            self.display_var.set(self.expression if self.expression else "0")
        elif event.keysym == "Escape":
            self.expression = ""
            self.display_var.set("0")

    def _evaluate(self):
        if not self.expression:
            return
        allowed = set("0123456789+-*/(). ")
        if any(ch not in allowed for ch in self.expression):
            self.display_var.set("오류")
            self.expression = ""
            return
        try:
            result = eval(self.expression, {"__builtins__": {}}, {})
            self.expression = str(result)
            self.display_var.set(self.expression)
        except Exception:
            self.display_var.set("오류")
            self.expression = ""


if __name__ == "__main__":
    app = Calculator()
    app.mainloop()
