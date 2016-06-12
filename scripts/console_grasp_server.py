#!/usr/bin/env python

import urwid, sys, os

'''
Console application for trianing and testing pick and place framework
Author: Luke Fraser
'''

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
        pass


def main():
    pass

if __name__ == '__main__':
    main()
