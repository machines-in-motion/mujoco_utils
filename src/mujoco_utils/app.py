"""
License: BSD 3-Clause License
Copyright (C) 2022, New York University
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import time

import numpy as np

import glfw

from dm_control import _render
from dm_control import mujoco
from dm_control.viewer import gui
from dm_control.viewer import renderer
from dm_control.viewer import viewer
from dm_control.viewer import views

_MAX_FRONTBUFFER_SIZE = 2048
_MAX_FRONTBUFFER_SIZE = 2048


class MujocoApplication:
    def __init__(self, physics, frame_rate=24, dt=0.001, use_touchpad=True, width=768, height=576, title="Mujoco Simulator"):
        self.physics = physics
        self.dt = dt

        self.viewport = renderer.Viewport(width, height)
        self.window = gui.RenderWindow(width, height, title)

        self.context = self.window._context

        self.viewer_layout = views.ViewportLayout()
        self.viewer = viewer.Viewer(
            self.viewport, self.window.mouse, self.window.keyboard)

        self.render_surface = _render.Renderer(
            max_width=_MAX_FRONTBUFFER_SIZE, max_height=_MAX_FRONTBUFFER_SIZE)
        self.renderer = renderer.OffScreenRenderer(
            physics.model, self.render_surface)
        self.renderer.components += self.viewer_layout
        self.viewer.initialize(physics, self.renderer, touchpad=use_touchpad)

        self.last_render = 0.
        self.render_interval = 1 / frame_rate # Rendering at 24 fps.

        self.draw_counter = 0
        self.step_counter = 0
        self.sleep_step = 0
        self.step_time = 0.
        self.first_time_sleep_duration_warning = True

    def step(self, sleep=False):
        if self.step_counter == 0:
            self.step_time = time.time()

        self.step_counter += 1

        # Apply the pertubations and step the physics.
        with self.viewer.perturbation.apply(False):
            self.physics.step()

        # Sleep for dt if desired.
        if sleep:
            if isinstance(sleep, bool):
                time.sleep(self.dt)
            else:
                steps_per_render = sleep * self.render_interval * 1. / self.dt

                if self.step_counter > steps_per_render + self.sleep_step:
                    self.sleep_step = self.step_counter

                    now = time.time()

                    # How long did we spending stepping before a redraw?
                    step_duration = now - self.step_time

                    self.last_render = now
                    self.redraw()
                    self.render_time = time.time() - now

                    sleep_duration = self.render_interval - self.render_time - step_duration
                    time.sleep(max(sleep_duration, 0.))

                    self.step_time = time.time()

                # Exiting early here as we do the rendering in the above if-block if desired.
                return

        # If enough time passed, do a rerender.
        now = time.time()
        if now - self.last_render > self.render_interval:
            self.redraw()
            self.render_time = time.time() - now
            self.last_render = now

    def redraw(self):
        self.draw_counter += 1

        if glfw.window_should_close(self.context.window):
            self.window.close()
            return

        # Update the viewport and perform the rendering.
        self.viewport.set_size(*self.window.shape)
        self.viewer.render()

        with self.context.make_current() as ctx:
            ctx.call(
                self.window._update_gui_on_render_thread, self.context.window, self.renderer.pixels)

        self.window.mouse.process_events()
        self.window.keyboard.process_events()

    def close(self):
        self.window.close()