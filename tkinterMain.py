
import sys

import tkinter as tk

from tkinter.filedialog import *


def main():
    root = tk.Tk()
    root.title("3D Objets Crypto-Compresser")
    root.tk.call('wm', 'iconphoto', root._w, tk.PhotoImage(file='/path/to/ico/icon.png'))
    # root.wm_iconbitmap('Images/cube-logo.png')

    label = tk.Label(root, text="Hello world")
    label.grid(row=0, column=0)
    label = tk.Label(root, text="Hello world1")
    label.grid(row=1, column=1)
    label = tk.Label(root, text="Hello world2")
    label.grid(row=1, column=0)
    label = tk.Label(root, text="Hello world3")
    label.grid(row=0, column=1)

    root.mainloop()


def test1():
    window = tk.Tk()

    filepath = askopenfilename(title="Select a model", filetypes=[('obj files', 'obj')], )
    filepath = askopenfilename(title="Ouvrir une image",filetypes=[('png files','.png'),('all files','.*')])
    photo = tk.PhotoImage(file=filepath)

    canvas = tk.Canvas(window, width=photo.width(), height=photo.height(), bg="yellow")
    canvas.create_image(0, 0, anchor=tk.NW, image=photo)
    canvas.pack()

    window.mainloop()


if __name__ == '__main__':
	sys.exit(main())