import argparse
import os
import sys

import pyglet
from pyglet.gui import Frame, PushButton

from pytongue.gui.widgets import Widget

import pytongue.system.mouthpieces as mp
from pytongue.window.window import AUTrackingWindow

DEFAULT_CELL_SIZE = 180

target_string = ["contract", "act", "expand", "left", "right"]


dirname = os.path.dirname(os.path.abspath(__file__))

pyglet.resource.path = [dirname + "/resources/images"]

## Calculate the speed of the cursor
def slidebar(input_value, limit=1.0):
    if abs(input_value) < 100:
        return input_value / 100
    else:
        return limit if input_value > 0 else -limit

class Data: 
    def __init__(self): 
        self.x = 0 
        self.y = 0 
        self.targ = None
        self.activation_time = None
        self.speed = None
        self.is_virgin = True
        self.location = () ## location of the first click
    
    def update(self, x, y, targ, time, speed, virgin, location):
        self.x = x
        self.y = y
        self.activation_time = time
        self.targ = targ
        self.speed = speed
        self.is_virgin = virgin
        self.location = location
        
    def publish(self): 
        print("X: {:.0f} , Y: {:.0f}".format(self.x, self.y))
        
        if self.targ is not None: 
            print("Target: ", target_string[self.targ])
        if not self.activation_time == None: 
            print("AT: ", self.activation_time)     
        if not self.speed == None:
            print("Speed: ", self.speed)       


class Target(Widget):
    def __init__(
        self,
        id: int, 
        x: int,
        y: int,
        width: int,
        height: int,
        activation_time: float,
        idle_color: tuple,
        active_color: tuple,
        border_color: tuple,
        highlight_color: tuple = None,
        batch: pyglet.graphics.Batch = None,
        group: pyglet.graphics.Group = None,
    ) -> None:
        self._batch = batch or pyglet.graphics.Batch()
        self.id = id
        self._active = False  # inactive by default
        self._idle_color = idle_color
        self._active_color = active_color
        self._border_color = border_color
        self._activation_time = activation_time
        self._timer = 0
        self.data = Data()

        self._location = self.data.location
        self._virgin = self.data.is_virgin
        self._speed = self.data.speed

        if highlight_color is None:
            highlight_color = idle_color
        self._highlight_color = highlight_color

        self._group = group

        self._centre = (x + width // 2, y + height // 2)

        self._body = pyglet.shapes.BorderedRectangle(
            x,
            y,
            width,
            height,
            border=2,
            color=idle_color,
            border_color=border_color,
            batch=self._batch,
            group=self._group,
        )

    def rescale(self):
        raise NotImplementedError

    def _asdict(self) -> dict:
        raise NotImplementedError

    def reset(self):
        self._body.visible = True
        self._body.color = self._idle_color

    def press(self, dt, x, y):    
        self._body.color = self._active_color
        if self._timer > self._activation_time:
            self._timer += dt
        else:
            self._timer += dt
    
    def get_time(self): 
        return self._timer
    
    def get_speed(self):
        return self._speed

    def get_virgin(self):
        return self._virgin

    def release(self):
        self._timer = 0
        if self._virgin or not self._on:
            self._speed = 0
            self._virgin = True
        self.reset()
        self.dispatch_event("on_release", self)

class Task(pyglet.event.EventDispatcher):
    def __init__(
        self, window, batch, group, cell_size=DEFAULT_CELL_SIZE
    ):
        """Create an instance of a Frame."""
        self._win = window
        self._size = window.size
        self._cell_size = cell_size
        self._pressed_target = None
        self._running = False
        self._batch = batch
        self._fg_group = pyglet.graphics.Group(order=1, parent=group)
        self._bg_group = pyglet.graphics.Group(order=0, parent=group)
        self._populate()
        self.data = Data()

    def _get_parameters(self):
        return self._cell_size, self._timeout

    def _hash(self, x, y):
        """Determine which custom cell (x, y) falls into"""
        for (cell_x, cell_y), button in self._cells.items():
            if cell_x <= x < cell_x + button._body.width and cell_y <= y < cell_y + button._body.height:
                return cell_x, cell_y
        return None  # Return None if the point is not within any cell


    def _populate(self):
        self._cells = {}
        id = 0

        # Define the specific cells
        cell_definitions = [
            (0, 180, 135, 360),  # Upper left: (x, y, width, height)
            (135, 180, 270, 360),  # Upper middle
            (405, 180, 135, 360),  # Upper right
            (0, 0, 270, 180),  # Lower left
            (270, 0, 270, 180),  # Lower right
        ]

        for x, y, width, height in cell_definitions:
            button = Target(
                id,
                x,
                y,
                width,
                height,
                activation_time=0,
                batch=self._batch,
                group=self._bg_group,
                idle_color=(240, 128, 128, 255),
                active_color=(205, 92, 92, 255),
                border_color=(139, 69, 69, 255),
                highlight_color=(255, 160, 122, 255),
            )
            id += 1
            button.push_handlers(on_activation=self.on_target_activation)

            # Store the button in the dictionary by its (x, y) hash
            self._cells[(x, y)] = button

        
    def on_target_activation(self, widget):
        widget.release()
        widget.deactivate()

    def update(self, dt):
        x = self._win.last_contact_event.x
        y = self._win.last_contact_event.y
        time = None
        id = None
        speed = None
        location = self.data.location
        virgin = self.data.is_virgin

        # Use the new _hash method to find the active widget
        cell_key = self._hash(x, y)
        widget = self._cells.get(cell_key, None)

        if widget is not None and self._win.last_contact_event.contact:
            if widget != self._pressed_target and self._pressed_target is not None:
                self._pressed_target.release()
            if virgin:
                virgin = False
                location = (x, y)
            widget.press(dt, x, y)
            time = widget.get_time()
            speed = slidebar(y - location[1])
            id = widget.id
            self._pressed_target = widget
        elif self._pressed_target is not None:
            self._pressed_target.release()
            virgin = True
            if widget is None:
                speed = 0
            else:
                speed = widget.get_speed()
            self._pressed_target = None
        self.data.update(x, y, id, time, speed, virgin, location)

        
    def run(self):
        self._running = True
        pyglet.clock.schedule_interval(self.update, 1 / 60) ## the rate of the update

    @property
    def running(self):
        return self._running

    @property
    def cell_size(self):
        return self._cell_size

    @cell_size.setter
    def cell_size(self, value):
        del self._cells, self._center_widget
        self._cell_size = value
        self._populate()

class SetupScreen:
    def __init__(
        self,
        window,
        batch,
        group,
        margin=(0.3, 0.2),
        hspace=0.05,
        vspace=0.05,
    ) -> None:
        self._group = group
        self._bg_group = pyglet.graphics.Group(order=0, parent=group)
        self._fg_group = pyglet.graphics.Group(order=1, parent=group)

        try:
            self.margin = (window.width * margin[0], window.height * margin[1])
        except TypeError:
            self.margin = (window.width * margin, window.height * margin)
        self.spacing = (window.width * hspace, window.height * vspace)
        self.top_corner = self.margin[0], window.height - self.margin[1]


        self.task_setup_frame = Frame(window)

        self.background = pyglet.shapes.Rectangle(
            0,
            0,
            window.width,
            window.height,
            color=(0, 0, 0, 200),
            group=self._bg_group,
            batch=batch,
        )
        pressed = pyglet.resource.image("start-button-pressed.png")
        self.start_button = PushButton(
            x=window.size[0] // 2 - pressed.width // 2,
            y=self.top_corner[1] - self.spacing[1] * 12,
            pressed=pressed,
            depressed=pyglet.resource.image("start-button-depressed.png"),
            batch=batch,
            group=self._fg_group,
        )
        self.task_setup_frame.add_widget(self.start_button)


if __name__ == "__main__":
    task_batch = pyglet.graphics.Batch()
    background = pyglet.graphics.Group(order=0)
    middleground = pyglet.graphics.Group(order=1)
    foreground = pyglet.graphics.Group(order=2)

    setup_batch = pyglet.graphics.Batch()

    parser = argparse.ArgumentParser(
        description="Test graphical interface configuration."
    )
    parser.add_argument(
        "-e",
        "--editable",
        action="store_true",
        help="allows users to edit the task parameters",
    )
    parser.add_argument(
        "-m",
        metavar="MOUTHPIECE",
        type=str,
        nargs="?",
        help="describes the mouthpiece PCB, flat or split",
        default="flat",
    )


    args = parser.parse_args()

    if args.m.lower() == "flat":
        mouthpiece = getattr(mp, "MP15")
    elif args.m.lower() == "split":
        mouthpiece = getattr(mp, "MP18")
    else:
        sys.exit(1)

    if getattr(sys, "frozen", False):
        application_path = os.path.dirname(sys.executable)
    elif __file__:
        application_path = os.path.dirname(__file__)


    gl_config = pyglet.gl.Config(sample_buffers=1, samples=4, alpha_size=8)
    win = AUTrackingWindow(
        sensor_positions=mouthpiece.sensor_positions,
        handle_mouse_events=True,
        invert_x=False,
        invert_y=False,
        width=540,
        height=540,
        caption="PyTongue GUI",
        style=pyglet.window.Window.WINDOW_STYLE_DEFAULT,
        resizable=False,
        config=gl_config,
    )
    
    # Create cursor
    win.set_mouse_visible(False)
    img = pyglet.resource.image("cursor.png")
    img.anchor_x, img.anchor_y = 0, img.height
    cursor = pyglet.window.ImageMouseCursor(img, 0, 0)
    win.set_mouse_cursor(cursor)
    cursor = pyglet.sprite.Sprite(img, batch=task_batch, group=middleground)


    task = Task(win, task_batch, background)
    recorder = None

    show_setup = True
    
    @win.event("on_draw")
    def on_draw():
        global show_setup 

        x = win.last_contact_event.x
        y = win.last_contact_event.y
        cursor.x, cursor.y = x, y

        win.clear()

        task_batch.draw()
        if show_setup:
            setup_batch.draw()
        else: 
            task.data.publish()
        win.set_mouse_visible(True)
        
    setup = SetupScreen(win, setup_batch, foreground)
    @setup.start_button.event("on_release")
    def start_task():
        global show_setup 
        show_setup = False
        win.pop_handlers()
        win.push_handlers(recorder)
        task.run()
        
    # start running the application
    pyglet.app.run()
