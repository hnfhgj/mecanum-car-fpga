import types,unittest
from inputs import HeldInput

class InputTests(unittest.TestCase):
 def make(self,fg,focus=None,pressed=()):
  obj=HeldInput.__new__(HeldInput)
  obj.root=types.SimpleNamespace(winfo_id=lambda:101,focus_get=lambda:focus)
  obj.user32=types.SimpleNamespace(GetAncestor=lambda h,k:100 if h in (101,100,200) else h,
   GetForegroundWindow=lambda:fg,GetAsyncKeyState=lambda k:0x8000 if k in pressed else 0)
  return obj
 def test_owned_popup_keeps_arm_context_but_sends_no_keys(self):
  obj=self.make(200,pressed=(ord('W'),));self.assertTrue(obj.foreground());self.assertEqual(obj.mask(),0)
 def test_foreign_window_is_not_console(self):
  obj=self.make(300,pressed=(ord('W'),));self.assertFalse(obj.foreground());self.assertEqual(obj.mask(),0)
 def test_combobox_edit_does_not_drive(self):
  obj=self.make(100,types.SimpleNamespace(winfo_class=lambda:'TCombobox'),(ord('W'),));self.assertEqual(obj.mask(),0)
 def test_focused_console_reads_keyboard_and_side_buttons(self):
  obj=self.make(100,pressed=(ord('W'),5));self.assertEqual(obj.mask(),1|32)
 def test_tcl_only_popup_does_not_resolve_python_widget(self):
  obj=self.make(100,pressed=(ord('W'),))
  def call(*args):
   if args==('focus',):return '.speed.popdown.f.l'
   if args[:2]==('winfo','class'):return 'Listbox'
   return '.speed.popdown'
  obj.root.tk=types.SimpleNamespace(call=call)
  obj.root.focus_get=lambda:(_ for _ in ()).throw(KeyError('popdown'))
  self.assertEqual(obj.mask(),0)

if __name__=='__main__':unittest.main(verbosity=2)

