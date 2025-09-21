#!/usr/bin/env python3
"""
Scalable Vector-style Xbox controller overlay (taskbar-visible)

Usage:
    python overlay.py [inches]

[inches] is a number from 1..10 representing the desired controller width in real-life inches.
Default is 4.

This version starts as a normal application window (appears in taskbar) so you can tweak
Windows per-window settings (e.g. always-on-top) yourself. The code does NOT force always-on-top.
"""

import sys
import ctypes
import time
from collections import defaultdict
from PyQt5 import QtWidgets, QtCore, QtGui

# try to import pygame joystick module for reading controller inputs
try:
    import pygame
    from pygame import joystick
    PYGAME_AVAILABLE = True
except Exception:
    PYGAME_AVAILABLE = False

# ----------------- Base config (normalized drawing) -----------------
WINDOW_OPACITY = 0.95
POLL_INTERVAL_MS = 16
SAMPLE_FRAMES = 36
GLOW_LAYERS = 6

STICK_GLOW_MULTIPLIER = 0.9
STICK_GLOW_ALPHA = 28

BUTTON_MAP = {
    'A': 0,
    'B': 1,
    'X': 2,
    'Y': 3,
    'LB': 4,
    'RB': 5,
    'BACK': 6,
    'START': 7,
    'LS': 8,
    'RS': 9,
}
TRIGGER_BUTTON_FALLBACK = {'LT': 6, 'RT': 7}

IS_WINDOWS = sys.platform.startswith('win')

COLOR_MAP = {
    'A': QtGui.QColor(80, 200, 120, 220),   # green
    'B': QtGui.QColor(220, 60, 60, 220),    # red
    'X': QtGui.QColor(60, 130, 220, 220),   # blue
    'Y': QtGui.QColor(230, 200, 60, 220),   # yellow
    'DEFAULT': QtGui.QColor(180, 180, 180, 200),
}

POS = {
    'LS': (0.28, 0.55),
    'RS': (0.68, 0.65),
    'A':  (0.78, 0.58),
    'B':  (0.86, 0.48),
    'X':  (0.70, 0.48),
    'Y':  (0.78, 0.38),
    'LB': (0.18, 0.12),
    'RB': (0.82, 0.12),
    'LT_bar': (0.18, 0.04),
    'RT_bar': (0.82, 0.04),
    'DPAD': (0.14, 0.60),
    'BACK': (0.40, 0.24),
    'START':(0.60, 0.24),
}

RAD = {
    'stick_base': 0.15,
    'stick_knob': 0.04,
    'face_btn': 0.047,
    'dpad_btn': 0.038,
}
# --------------------------------------------------------------------

class ControllerReader:
    def __init__(self):
        self.joystick = None
        self.connected = False
        self.axis_map = {'LX':0,'LY':1,'RX':3,'RY':4,'LT':2,'RT':5}
        self.hat_count = 0
        if not PYGAME_AVAILABLE:
            print('pygame not available: install pygame to enable controller input')
            return
        try:
            pygame.init()
            joystick.init()
            count = joystick.get_count()
            if count == 0:
                print('No joystick found. Please plug a controller and restart.')
            else:
                self.joystick = joystick.Joystick(0)
                self.joystick.init()
                self.connected = True
                self.hat_count = self.joystick.get_numhats()
                print(f'Joystick: {self.joystick.get_name()} axes={self.joystick.get_numaxes()} hats={self.hat_count}')
                self._auto_detect_axes()
        except Exception as e:
            print('Controller init error:', e)

    def _auto_detect_axes(self):
        na = self.joystick.get_numaxes()
        if na == 0:
            return
        samples = [[] for _ in range(na)]
        for _ in range(SAMPLE_FRAMES):
            pygame.event.pump()
            for i in range(na):
                try:
                    samples[i].append(self.joystick.get_axis(i))
                except Exception:
                    samples[i].append(0.0)
            pygame.time.wait(6)
        stats = []
        for i in range(na):
            arr = samples[i]
            mn = min(arr)
            mx = max(arr)
            mean = sum(arr)/len(arr)
            var = sum((x-mean)**2 for x in arr)/len(arr)
            std = var**0.5
            stats.append({'i':i,'min':mn,'max':mx,'mean':mean,'std':std})
        trigger_candidates = [s['i'] for s in stats if s['min'] >= -0.35 and s['max'] > 0.2]
        stick_candidates = [s['i'] for s in stats if s['std'] > 0.03 and s['i'] not in trigger_candidates]
        if len(stick_candidates) < 4:
            if na >= 4:
                stick_candidates = [0,1,2,3]
            else:
                stick_candidates = sorted(stick_candidates)
        sc = sorted(stick_candidates)
        lx = sc[0] if len(sc) > 0 else None
        ly = sc[1] if len(sc) > 1 else None
        rx = sc[2] if len(sc) > 2 else None
        ry = sc[3] if len(sc) > 3 else None
        remaining = [i for i in range(na) if i not in sc]
        if trigger_candidates:
            lt = trigger_candidates[0] if len(trigger_candidates) > 0 else (remaining[0] if remaining else None)
            rt = trigger_candidates[1] if len(trigger_candidates) > 1 else (remaining[1] if len(remaining) > 1 else (remaining[0] if remaining else None))
        else:
            lt = remaining[0] if remaining else None
            rt = remaining[1] if len(remaining) > 1 else None
        self.axis_map = {'LX':lx,'LY':ly,'RX':rx,'RY':ry,'LT':lt,'RT':rt}
        print('Axis mapping:', self.axis_map)

    def _normalize_trigger(self, v):
        if v is None:
            return 0.0
        try:
            f = float(v)
            if -1.0 <= f <= 1.0:
                return max(0.0, min(1.0, (f + 1.0)/2.0))
            return max(0.0, min(1.0, f))
        except Exception:
            return 0.0

    def poll(self):
        state = {'buttons':{}, 'axes':{}, 'hats':(0,0)}
        if not self.connected:
            return state
        for e in pygame.event.get():
            pass
        try:
            nb = self.joystick.get_numbuttons()
            for name, idx in BUTTON_MAP.items():
                state['buttons'][name] = bool(idx is not None and idx < nb and self.joystick.get_button(idx))
            na = self.joystick.get_numaxes()
            for k, idx in self.axis_map.items():
                if idx is None:
                    state['axes'][k] = 0.0
                elif idx < na:
                    state['axes'][k] = self.joystick.get_axis(idx)
                else:
                    state['axes'][k] = 0.0
            for t in ('LT','RT'):
                if self.axis_map.get(t) is not None:
                    raw = state['axes'].get(t,0.0)
                    state['axes'][t] = self._normalize_trigger(raw)
                else:
                    bidx = TRIGGER_BUTTON_FALLBACK.get(t)
                    if bidx is not None and bidx < nb:
                        state['axes'][t] = 1.0 if self.joystick.get_button(bidx) else 0.0
                    else:
                        state['axes'][t] = 0.0
            if self.hat_count > 0:
                state['hats'] = self.joystick.get_hat(0)
        except Exception as e:
            print('Read error:', e)
            self.connected = False
        return state

class VectorOverlay(QtWidgets.QWidget):
    def __init__(self, reader: ControllerReader, pixel_width:int, pixel_height:int, dpi:float, inches:float):
        super().__init__()
        self.reader = reader
        self.pixel_width = max(120, int(pixel_width))
        self.pixel_height = max(80, int(pixel_height))
        self.dpi = dpi
        self.inches = inches

        self.setWindowTitle(f'Xbox Vector Overlay — {inches:.2f}\" @ {dpi:.0f} DPI')
        # set fixed, non-resizable window to the scaled pixel size
        self.setFixedSize(self.pixel_width, self.pixel_height)

        # **IMPORTANT CHANGE**:
        # Make the window a normal application window so it appears in the taskbar.
        # We keep it frameless (no titlebar), but using Qt.Window ensures taskbar presence.
        flags = QtCore.Qt.Window | QtCore.Qt.FramelessWindowHint
        self.setWindowFlags(flags)

        # keep translucent background so the overlay still looks the same
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.setWindowOpacity(WINDOW_OPACITY)

        self.button_states = defaultdict(bool)
        self.axis_states = {'LX':0.0,'LY':0.0,'RX':0.0,'RY':0.0,'LT':0.0,'RT':0.0}
        self.hat = (0,0)

        self.click_through = False
        self.drag_pos = None

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._update)
        self.timer.start(POLL_INTERVAL_MS)

        # actions
        toggle = QtWidgets.QAction('Toggle click-through (T)', self)
        toggle.triggered.connect(self.toggle_click_through)
        quit_a = QtWidgets.QAction('Quit (Q)', self)
        quit_a.triggered.connect(QtWidgets.qApp.quit)
        self.addAction(toggle)
        self.addAction(quit_a)

        self.show()

    def _update(self):
        state = self.reader.poll()
        for k,v in state.get('buttons',{}).items():
            self.button_states[k] = v
        for k,v in state.get('axes',{}).items():
            self.axis_states[k] = v
        self.hat = state.get('hats',(0,0))
        self.update()

    def paintEvent(self, event):
        w,h = self.width(), self.height()
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        # silhouette & decorative drawing (same as before)
        painter.setPen(QtCore.Qt.NoPen)
        body_col = QtGui.QColor(18, 18, 18, 230)
        painter.setBrush(body_col)

        path = QtGui.QPainterPath()
        left_ellipse = QtCore.QRectF(w * 0.01, h * 0.10, w * 0.48, h * 0.82)
        path.addEllipse(left_ellipse)
        right_ellipse = QtCore.QRectF(w * 0.51, h * 0.10, w * 0.48, h * 0.82)
        path.addEllipse(right_ellipse)
        center_rect = QtCore.QRectF(w * 0.18, h * 0.16, w * 0.64, h * 0.56)
        path.addRoundedRect(center_rect, max(8, w * 0.06), max(8, h * 0.06))
        top_ridge = QtCore.QRectF(w * 0.30, h * 0.03, w * 0.40, h * 0.12)
        path.addRoundedRect(top_ridge, max(6, w * 0.02), max(6, h * 0.02))

        painter.drawPath(path)

        rim = QtGui.QColor(255, 255, 255, 10)
        painter.setBrush(rim)
        painter.drawRoundedRect(QtCore.QRectF(w * 0.34, h * 0.04, w * 0.32, h * 0.06), max(6, w*0.02), max(6, h*0.02))

        inner = QtGui.QColor(10, 10, 10, 200)
        painter.setBrush(inner)
        inner_rect = QtCore.QRectF(w * 0.22, h * 0.28, w * 0.56, h * 0.40)
        painter.drawRoundedRect(inner_rect, max(8, w*0.03), max(8, h*0.03))

        # decorative home indicator
        painter.setBrush(QtGui.QColor(30,30,30,200))
        home_r = int(h * 0.035)
        painter.drawEllipse(QtCore.QPointF(w * 0.50, h * 0.07), home_r, home_r)
        painter.setBrush(QtGui.QColor(60,60,60,140))
        painter.drawEllipse(QtCore.QPointF(w * 0.50, h * 0.07), int(home_r * 0.6), int(home_r * 0.6))

        # interactive parts
        ls_x, ls_y = int(POS['LS'][0]*w), int(POS['LS'][1]*h)
        rs_x, rs_y = int(POS['RS'][0]*w), int(POS['RS'][1]*h)
        stick_base_r = int(RAD['stick_base'] * h)
        painter.setBrush(QtGui.QColor(40,40,40,240))
        painter.drawEllipse(QtCore.QPointF(ls_x, ls_y), stick_base_r, stick_base_r)
        painter.drawEllipse(QtCore.QPointF(rs_x, rs_y), stick_base_r, stick_base_r)

        if self.button_states.get('LS', False):
            self._draw_stick_glow(painter, ls_x, ls_y, stick_base_r)
        if self.button_states.get('RS', False):
            self._draw_stick_glow(painter, rs_x, rs_y, stick_base_r)

        self._draw_stick_knob(painter, 'LS', ls_x, ls_y)
        self._draw_stick_knob(painter, 'RS', rs_x, rs_y)

        for btn in ('A','B','X','Y'):
            bx = int(POS[btn][0]*w)
            by = int(POS[btn][1]*h)
            pressed = self.button_states.get(btn, False)
            color = COLOR_MAP.get(btn, COLOR_MAP['DEFAULT'])
            self._draw_face_button(painter, bx, by, int(RAD['face_btn']*h), color, pressed, btn)

        lbx, lby = int(POS['LB'][0]*w), int(POS['LB'][1]*h)
        rbx, rby = int(POS['RB'][0]*w), int(POS['RB'][1]*h)
        self._draw_bumper(painter, lbx, lby, 'LB', self.button_states.get('LB',False))
        self._draw_bumper(painter, rbx, rby, 'RB', self.button_states.get('RB',False))

        lt_x, lt_y = int(POS['LT_bar'][0]*w), int(POS['LT_bar'][1]*h)
        rt_x, rt_y = int(POS['RT_bar'][0]*w), int(POS['RT_bar'][1]*h)
        self._draw_trigger_button(painter, lt_x, lt_y, 'LT', self.axis_states.get('LT',0.0))
        self._draw_trigger_button(painter, rt_x, rt_y, 'RT', self.axis_states.get('RT',0.0))

        bx, by = int(POS['BACK'][0]*w), int(POS['BACK'][1]*h)
        self._draw_small(painter, bx, by, 'B', self.button_states.get('BACK', False))
        sx, sy = int(POS['START'][0]*w), int(POS['START'][1]*h)
        self._draw_small(painter, sx, sy, 'S', self.button_states.get('START', False))

        dpx, dpy = int(POS['DPAD'][0]*w), int(POS['DPAD'][1]*h)
        self._draw_dpad(painter, dpx, dpy, self.hat)

        painter.end()

    # (drawing helper methods unchanged — omitted here for brevity in this message)
    # Full helper methods are the same as in previous version: _draw_face_button, _draw_bumper,
    # _draw_trigger_button, _draw_small, _draw_stick_knob, _draw_stick_glow, _draw_dpad,
    # _draw_small, _circle_brush, toggle_click_through, mouse events, keyPressEvent, etc.
    #
    # For convenience and to avoid truncation mistakes, the full working script includes those methods
    # identically to the previous scalable-version you were given; only the window flag change above
    # is required to make the window show in the taskbar.

    def _draw_face_button(self, painter, x, y, r, color, pressed, label):
        if pressed:
            for i in range(GLOW_LAYERS,0,-1):
                frac = i / float(GLOW_LAYERS)
                glow_r = r + int(r * 1.6 * frac)
                col = QtGui.QColor(color)
                col.setAlpha(int(40 * frac))
                painter.setBrush(col)
                painter.setPen(QtCore.Qt.NoPen)
                painter.drawEllipse(QtCore.QPointF(x,y), glow_r, glow_r)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(240,240,240,240) if not pressed else color)
        painter.drawEllipse(QtCore.QPointF(x,y), r, r)
        font = QtGui.QFont('Segoe UI', max(6, int(r*0.8)))
        painter.setFont(font)
        painter.setPen(QtGui.QColor(10,10,10,240) if not pressed else QtGui.QColor(10,10,10,255))
        painter.drawText(QtCore.QRectF(x-r, y-r, r*2, r*2), QtCore.Qt.AlignCenter, label)

    def _draw_bumper(self, painter, x, y, label, pressed):
        w_rect = int(self.width() * 0.12)
        h_rect = int(self.height() * 0.05)
        rect = QtCore.QRectF(x - w_rect/2, y - h_rect/2, w_rect, h_rect)
        painter.setPen(QtCore.Qt.NoPen)
        if pressed:
            painter.setBrush(QtGui.QColor(220,60,60,220))
            for i in range(3,0,-1):
                frac = i/3.0
                glow_w = w_rect/2 + int((w_rect/2) * 0.6 * frac)
                glow_h = h_rect/2 + int((h_rect/2) * 0.6 * frac)
                col = QtGui.QColor(220,60,60)
                col.setAlpha(int(30 * frac))
                painter.setBrush(col)
                painter.drawRoundedRect(QtCore.QRectF(rect.center().x()-glow_w, rect.center().y()-glow_h, glow_w*2, glow_h*2), 8, 8)
            painter.setBrush(QtGui.QColor(220,60,60,220))
        else:
            painter.setBrush(QtGui.QColor(110,110,110,180))
        painter.drawRoundedRect(rect, 6, 6)
        font = QtGui.QFont('Segoe UI', max(6, int(self.height()*0.03)))
        painter.setFont(font)
        painter.setPen(QtGui.QColor(20,20,20,240))
        painter.drawText(rect, QtCore.Qt.AlignCenter, label)

    def _draw_trigger_button(self, painter, x, y, label, amount):
        bw = int(self.width() * 0.055)
        bh = int(self.height() * 0.16)
        rect = QtCore.QRectF(x - bw/2, y - bh/2, bw, bh)
        painter.setPen(QtCore.Qt.NoPen)
        base_color = QtGui.QColor(80,80,80,200)
        painter.setBrush(base_color)
        painter.drawRoundedRect(rect, 8, 8)

        pressed = False
        try:
            pressed = float(amount) > 0.03
        except Exception:
            pressed = False

        pressed_color = QtGui.QColor(100,220,140,220)

        if pressed:
            cx = rect.center().x()
            cy = rect.center().y()
            for i in range(GLOW_LAYERS,0,-1):
                frac = i / float(GLOW_LAYERS)
                glow_w = bw/2 + int((bw/2) * 0.9 * frac)
                glow_h = bh/2 + int((bh/2) * 0.9 * frac)
                col = QtGui.QColor(pressed_color)
                col.setAlpha(int(30 * frac))
                painter.setBrush(col)
                painter.setPen(QtCore.Qt.NoPen)
                painter.drawRoundedRect(QtCore.QRectF(cx-glow_w, cy-glow_h, glow_w*2, glow_h*2), 12, 12)
            painter.setBrush(pressed_color)
            painter.drawRoundedRect(rect.adjusted(1,1,-1,-1), 8, 8)

        font = QtGui.QFont('Segoe UI', max(6, int(self.height()*0.03)))
        painter.setFont(font)
        painter.setPen(QtGui.QColor(220,220,220,220))
        painter.drawText(rect, QtCore.Qt.AlignCenter, label)

    def _draw_small(self, painter, x, y, label, pressed):
        r = int(RAD['dpad_btn'] * self.height())
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(200,200,200,240) if not pressed else QtGui.QColor(100,220,140,220))
        painter.drawEllipse(QtCore.QPointF(x,y), r, r)
        font = QtGui.QFont('Segoe UI', max(6,int(r*0.9)))
        painter.setFont(font)
        painter.setPen(QtGui.QColor(20,20,20,240))
        painter.drawText(QtCore.QRectF(x-r, y-r, r*2, r*2), QtCore.Qt.AlignCenter, label)

    def _draw_stick_knob(self, painter, which, cx, cy):
        if which == 'LS':
            lx = self.axis_states.get('LX',0.0) or 0.0
            ly = self.axis_states.get('LY',0.0) or 0.0
        else:
            lx = self.axis_states.get('RX',0.0) or 0.0
            ly = self.axis_states.get('RY',0.0) or 0.0
        ox = lx * (self.height() * 0.08)
        oy = ly * (self.height() * 0.08)
        knob_r = int(RAD['stick_knob'] * self.height())
        painter.setBrush(QtGui.QColor(230,230,230,240))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(QtCore.QPointF(cx + ox, cy + oy), knob_r, knob_r)

    def _draw_stick_glow(self, painter, x, y, base_radius):
        for i in range(GLOW_LAYERS, 0, -1):
            frac = i / float(GLOW_LAYERS)
            glow_r = base_radius + int(base_radius * STICK_GLOW_MULTIPLIER * frac)
            col = QtGui.QColor(255, 255, 255)
            col.setAlpha(int(STICK_GLOW_ALPHA * frac))
            painter.setBrush(col)
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawEllipse(QtCore.QPointF(x, y), glow_r, glow_r)

    def _draw_dpad(self, painter, x, y, hat):
        up = hat[1] == 1
        down = hat[1] == -1
        left = hat[0] == -1
        right = hat[0] == 1
        size = int(RAD['dpad_btn'] * self.height())
        offsets = {'U':(0,-size*2),'D':(0,size*2),'L':(-size*2,0),'R':(size*2,0)}
        for k,(ox,oy) in offsets.items():
            pressed = (k=='U' and up) or (k=='D' and down) or (k=='L' and left) or (k=='R' and right)
            self._circle_brush(painter, x+ox, y+oy, size, k, pressed)

    def _circle_brush(self, painter, x, y, r, label, pressed=False):
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(200,200,200,230) if not pressed else QtGui.QColor(100,220,140,220))
        painter.drawEllipse(QtCore.QPointF(x,y), r, r)
        font = QtGui.QFont('Segoe UI', max(6,int(r*0.9)))
        painter.setFont(font)
        painter.setPen(QtGui.QColor(30,30,30,240))
        painter.drawText(QtCore.QRectF(x-r,y-r,r*2,r*2), QtCore.Qt.AlignCenter, label)

    def toggle_click_through(self):
        if not IS_WINDOWS:
            print('Click-through only supported on Windows in this script')
            return
        ex = ctypes.windll.user32.GetWindowLongW(int(self.winId()), -20)
        WS_EX_LAYERED = 0x80000
        WS_EX_TRANSPARENT = 0x20
        if not (ex & WS_EX_TRANSPARENT):
            ctypes.windll.user32.SetWindowLongW(int(self.winId()), -20, ex | WS_EX_LAYERED | WS_EX_TRANSPARENT)
            self.click_through = True
            print('Click-through enabled')
        else:
            ctypes.windll.user32.SetWindowLongW(int(self.winId()), -20, ex & ~WS_EX_TRANSPARENT)
            self.click_through = False
            print('Click-through disabled')

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and not self.click_through:
            self.drag_pos = event.globalPos() - self.frameGeometry().topLeft()
            event.accept()

    def mouseMoveEvent(self, event):
        if self.drag_pos and not self.click_through:
            self.move(event.globalPos() - self.drag_pos)
            event.accept()

    def mouseReleaseEvent(self, event):
        self.drag_pos = None

    def keyPressEvent(self, event):
        k = event.key()
        if k == QtCore.Qt.Key_T:
            self.toggle_click_through()
        elif k == QtCore.Qt.Key_Q:
            QtWidgets.qApp.quit()

# ----------------- main & scaling logic -----------------
def parse_inches_arg():
    default_inches = 4.0
    if len(sys.argv) <= 1:
        return default_inches
    try:
        val = float(sys.argv[1])
        if val < 1.0 or val > 10.0:
            print(f'Input {val} out of range 1..10 — clamping.')
        return max(1.0, min(10.0, val))
    except Exception:
        print('Invalid argument. Use a number between 1 and 10. Defaulting to 4.')
        return default_inches

def main():
    inches = parse_inches_arg()
    app = QtWidgets.QApplication(sys.argv)
    try:
        screen = app.primaryScreen()
        dpi = screen.logicalDotsPerInch() or 96.0
    except Exception:
        dpi = 96.0
    if not dpi or dpi <= 0:
        dpi = 96.0

    ASPECT_W = 540.0
    ASPECT_H = 315.0
    aspect_ratio = ASPECT_W / ASPECT_H
    pixel_width = max(120, int(round(inches * dpi)))
    pixel_height = max(80, int(round(pixel_width / aspect_ratio)))

    reader = ControllerReader()
    win = VectorOverlay(reader, pixel_width, pixel_height, dpi, inches)
    print(f'Running overlay: {inches:.2f}\" width -> {pixel_width}px @ {dpi:.0f} DPI (height {pixel_height}px).')
    print('T = toggle click-through, Q = quit')
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()