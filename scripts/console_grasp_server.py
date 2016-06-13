#!/usr/bin/env python

import urwid, sys, os

'''
Console application for trianing and testing pick and place framework
Author: Luke Fraser
'''
class ObjectWalker(urwid.ListWalker):
    def __init__(self, object_dict):
        self.object_reference = object_dict
    def _get_at_pos(self, pos):
        pass
class TrainingView(urwid.WidgetWrap):
    '''
    A Clss for displaying the interface.
    '''
    palette = [
        ('body',            'black',        'light gray',   'standout'),
        ('header',          'white',        'dark read',    'bold'),
        ('sceen edge',      'light blue',   'dark cyan'),
        ('main shadow',     'dark gray',    'black'),
        ('line',            'black',        'light gray',   'standout'),
        ('bg background',   'light gray',   'black'),
        ('bg 1',            'black',        'dark blue',    'standout'),
        ('bg 1 smooth',     'dark blue',    'black'),
        ('bg 2',            'black',        'dark cyan',    'standout'),
        ('bg 2 smooth',     'dark cyan',    'black'),
        ('button normal',   'light gray',   'dark blue',    'standout'),
        ('button select',   'white',        'dark green'),
        ('line',            'black',        'light gray',   'standout'),
        ('pg normal',       'white',        'black',        'standout'),
        ('pg complete',     'white',        'dark magenta'),
        ('pg smooth',       'dark magenta', 'black')
    ]
    def __init__(self):
        self.objects_ = {}
        urwid.WidgetWrap.__init__(self, self.MainWindow())
    def OnTrain(self):
        pass
    def OnLoad(self):
        pass
    def OnSave(self):
        pass
    def OnTest(self, object):
        pass
    def _ShadowWindow(view):
        bg     = urwid.AttrWrap(urwid.SolidFill(u"\u2592"), 'screen edge')
        shadow = urwid.AttrWrap(urwid.SolidFill(u" "), 'main shadow')

        bg = urwid.Overlay(shadow, bg,
            ('fixed left', 3), ('fixed right',  1),
            ('fixed top',  2), ('fixed bottom', 1))
        view = urwid.Overlay(view, bg,
            ('fixed left', 2), ('fixed right',  3),
            ('fixed top',  1), ('fixed bottom', 2))
        return view
    def _GenerateObjectList(self):
        self.objects_list_walker = ObjectWalker(self.objects_)
        return urwid.ListBox(self.objects_list_walker)

    def MainWindow(self):
        # Create Main Window

        # Add Components

        # Generate List of objects
        self.objects_view = self._GenerateObjectList()

        # Generate Options list
        self.options_view = self._GenerateOptions()
        view = urwid.Pile([
            ('weight', 2, self.objects_view),
            ('weight', 1, self.options_view)],
            focus_item=2)
        
        return self._ShadowWindow(view)

def main():
    pass

if __name__ == '__main__':
    main()
