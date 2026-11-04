"""R7 console for the native FPGA navigation chain in firmware 97F4."""
import argparse
import tkinter as tk
from tkinter import ttk

from legacy_console import App as BoardApp, ACCENT, BG, CARD, MUTED, SIDE, SMALL
from protocol import CALIBRATE, DRIVE, GO, GOAL, POINT, ROUTE


class App(BoardApp):
    def __init__(self, root, demo=False, enumerate_ports=True):
        super().__init__(root, demo, enumerate_ports)
        root.title('麦轮任务控制台 · 97F4 / R7 · FPGA 板载导航' + (' · 离线演示' if demo else ''))

    def _radar(self):
        super()._radar()
        side = next(widget for widget in self.pages['radar'].winfo_children()
                    if isinstance(widget, ttk.Notebook))
        side.tab(side.tabs()[1], text='任务4 · FPGA 导航')
        manual = self.root.nametowidget(side.tabs()[0]).winfo_children()[0]
        held_state = str(self.variable('held_state'))
        before = next(widget for widget in manual.winfo_children()
                      if isinstance(widget, tk.Label) and str(widget.cget('textvariable')) == held_state)
        keys = tk.Frame(manual, bg=CARD)
        keys.pack(fill='x', pady=(0, 7), before=before)
        self.key_indicators = {}
        self.key_display_mask = None
        for column in range(4):
            keys.grid_columnconfigure(column, weight=1, uniform='held_key')
        positions = (
            (1, 'W ↑', 0, 0, 1), (4, 'A ←', 0, 1, 1),
            (2, 'S ↓', 0, 2, 1), (8, 'D →', 0, 3, 1),
            (32, '侧键1 · 顺时针', 1, 0, 2), (16, '侧键2 · 逆时针', 1, 2, 2),
        )
        for bit, text, row, column, span in positions:
            label = tk.Label(keys, text=text, bg=SIDE, fg=MUTED, font=SMALL,
                             pady=4, bd=1, relief='solid', takefocus=False)
            label.grid(row=row, column=column, columnspan=span, sticky='ew', padx=2, pady=2)
            self.key_indicators[bit] = label
        self.value_label(keys, 'pressed_keys', '当前按键：未按下', font=SMALL, color=ACCENT).grid(
            row=2, column=0, columnspan=4, sticky='w', pady=(3, 0))

    def update_key_display(self):
        mask = self.held_input.mask()
        if mask == self.key_display_mask:
            return
        self.key_display_mask = mask
        for bit, label in self.key_indicators.items():
            pressed = bool(mask & bit)
            label.configure(bg=ACCENT if pressed else SIDE, fg=BG if pressed else MUTED)
        names = [name for bit, name in ((1, 'W'), (4, 'A'), (2, 'S'), (8, 'D'),
                                       (32, '侧键1'), (16, '侧键2')) if mask & bit]
        self.variable('pressed_keys').set('当前按键：' + (' + '.join(names) or '未按下'))

    def _navigation(self):
        super()._navigation()
        map_card = self.worlds[-1].master
        self.label(
            map_card, 'FPGA 建系、定位与寻路 · 场地边长 80～300 cm',
            color=ACCENT, size=9, wraplength=440, justify='left',
        ).pack(anchor='w', pady=(6, 0))

    def send(self, commands):
        operations = {command[0] for command in commands}
        if operations & {CALIBRATE, GOAL, GO, POINT, ROUTE}:
            role = ('go' if operations & {GO, ROUTE} else
                    'calibrate' if CALIBRATE in operations else 'goal')
            if not self.available(role):
                self.note('板端尚未就绪，请先停止遥控并检查定位状态。', True)
                return
            self.manual_armed.set(False)
            self.link.set_drive(0)
            if commands[0] != (DRIVE, 0, 0, 0):
                commands = [(DRIVE, 0, 0, 0), *commands]
        super().send(commands)

    def _poll_drive_once(self):
        self.update_key_display()
        super()._poll_drive_once()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true')
    parser.add_argument('--smoke', action='store_true')
    args = parser.parse_args()
    root = tk.Tk()
    if args.smoke:
        root.withdraw()
    app = App(root, args.demo, not args.smoke and not args.demo)
    if args.smoke:
        for page in app.pages:
            app.show(page)
            root.update_idletasks()
        assert app.link.thread is None
        assert app.link.actions.empty()
        root.destroy()
        print('PASS FPGA console smoke: all pages, no serial or motion')
    else:
        root.mainloop()


if __name__ == '__main__':
    main()
