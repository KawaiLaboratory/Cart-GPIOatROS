#!/usr/bin/env python
#-*- coding: utf-8 -*-

import roslib
import rospy

from kivy.app import App
from kivy.uix.widget import Widget

from kivy.properties import StringProperty

class TextWidget(Widget):
    text = StringProperty()

    def __init__(self, **kwargs):
        super(TextWidget, self).__init__(**kwargs)
        self.text = 'OFF'
        self.flg = False

    def buttonClicked(self):
      self.flg = not(self.flg)
      if self.flg:
        self.text = "ON"
      else:
        self.text = "OFF"

class DesignApp(App):
  def __init__(self, **kwargs):
    super(DesignApp, self).__init__(**kwargs)

  def build(self):
    return TextWidget()

if __name__ == '__main__':
   DesignApp().run()
