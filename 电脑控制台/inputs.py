"""Read held keys/side buttons only while this console owns foreground focus."""
import sys

class HeldInput:
 def emergency_stop(self):
  if self.user32:return bool(self.user32.GetAsyncKeyState(0x1B)&0x8000)
  return 'escape' in self.keys
 def __init__(self,root):
  self.root=root;self.keys=set();self.user32=None
  if sys.platform=='win32':
   import ctypes
   from ctypes import wintypes
   self.user32=ctypes.WinDLL('user32',use_last_error=True)
   self.user32.GetForegroundWindow.restype=wintypes.HWND
   self.user32.GetAncestor.argtypes=[wintypes.HWND,wintypes.UINT];self.user32.GetAncestor.restype=wintypes.HWND
   self.user32.GetAsyncKeyState.argtypes=[ctypes.c_int];self.user32.GetAsyncKeyState.restype=ctypes.c_short
  else:
   root.bind('<KeyPress>',lambda e:self.keys.add(e.keysym.lower()),add='+')
   root.bind('<KeyRelease>',lambda e:self.keys.discard(e.keysym.lower()),add='+')
 def foreground(self):
  if self.user32:
   hwnd=self.user32.GetAncestor(self.root.winfo_id(),2)
   foreground=self.user32.GetForegroundWindow()
   # Keep the arm switch across our own combobox popup, but never send keys
   # while a popup/modal owns foreground (mask() checks the actual root).
   return hwnd==foreground or hwnd==self.user32.GetAncestor(foreground,3)
  return self.root.focus_displayof() is not None
 def mask(self):
  if not self.foreground():return 0
  if self.user32 and self.user32.GetAncestor(self.root.winfo_id(),2)!=self.user32.GetForegroundWindow():return 0
  try:
   if hasattr(self.root,'tk'):
    # ttk creates its popup/listbox in Tcl, outside Python's widget registry.
    # focus_get() would raise KeyError when resolving such a widget path.
    name=str(self.root.tk.call('focus'))
    if name:
     klass=str(self.root.tk.call('winfo','class',name))
     top=str(self.root.tk.call('winfo','toplevel',name))
     if klass in ('Entry','TEntry','TSpinbox','Text','TCombobox','Listbox','TListbox') or top!=str(self.root):return 0
   else:
    focus=self.root.focus_get()
    if focus is not None and focus.winfo_class() in ('Entry','TEntry','TSpinbox','Text','TCombobox','Listbox','TListbox'):return 0
  except Exception:return 0
  if self.user32:
   codes=[ord('W'),ord('S'),ord('A'),ord('D'),6,5]
   return sum(1<<i for i,key in enumerate(codes) if self.user32.GetAsyncKeyState(key)&0x8000)
  return sum(1<<i for i,key in enumerate(['w','s','a','d','e','q']) if key in self.keys)

