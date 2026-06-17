from __future__ import annotations

import arcade

from graph import Graph
from models import ZoneType


SCREEN_WIDTH = 1900
SCREEN_HEIGHT = 500
SCREEN_TITLE = "Fly-in Drones"

NODE_RADIUS = 18
DRONE_RADIUS = 8

TURN_DURATION = 1.0
MARGIN = 20


COLOR_MAP = {
    "red": arcade.color.RED,
    "green": arcade.color.GREEN,
    "blue": arcade.color.BLUE,
    "yellow": arcade.color.YELLOW,
    "gray": arcade.color.GRAY,
    "grey": arcade.color.GRAY,
    "cyan": arcade.color.CYAN,
    "magenta": arcade.color.MAGENTA,
    "orange": arcade.color.ORANGE,
}


class Drone:
    def __init__(
        self,
        drone_id: int,
        path: list[tuple[str, int]],
    ) -> None:
        self.drone_id = drone_id
        self.path = sorted(path, key=lambda p: p[1])

        self.current_step = 0

        self.x = 0.0
        self.y = 0.0

        self.start_x = 0.0
        self.start_y = 0.0

        self.target_x = 0.0
        self.target_y = 0.0

        self.delivered = False
        self.pending_delivery = False


class DroneVisualizer(arcade.Window):

    def __init__(
        self,
        graph: Graph,
        paths: list[list[tuple[str, int]]],
        nb_drones: int,
    ) -> None:

        super().__init__(
            SCREEN_WIDTH,
            SCREEN_HEIGHT,
            SCREEN_TITLE,
        )

        arcade.set_background_color(
            arcade.color.BLACK
        )

        self.graph = graph
        self.nb_drones = nb_drones

        self.current_turn = 0
        self.elapsed = 0.0
        self.paused = False

        self.zone_positions = self._compute_layout()

        self.max_turn = 0

        self.drones: list[Drone] = []

        for drone_id, path in enumerate(paths, start=1):

            self.max_turn = max(
                self.max_turn,
                max(turn for _, turn in path),
            )

            drone = Drone(
                drone_id,
                path,
            )

            start_zone = path[0][0]

            x, y = self.zone_positions[start_zone]

            drone.x = x
            drone.y = y

            drone.start_x = x
            drone.start_y = y

            drone.target_x = x
            drone.target_y = y

            self.drones.append(drone)

    # ---------------------------------------------------------
    # Layout
    # ---------------------------------------------------------

    def _compute_layout(
        self,
    ) -> dict[str, tuple[float, float]]:

        xs = [z.x for z in self.graph.zones.values()]
        ys = [z.y for z in self.graph.zones.values()]

        min_x = min(xs)
        max_x = max(xs)

        min_y = min(ys)
        max_y = max(ys)

        graph_w = max(max_x - min_x, 1)
        graph_h = max(max_y - min_y, 1)

        scale_x = (
            SCREEN_WIDTH - 2 * MARGIN
        ) / graph_w

        scale_y = (
            SCREEN_HEIGHT - 2 * MARGIN
        ) / graph_h

        scale = min(scale_x, scale_y)

        positions: dict[str, tuple[float, float]] = {}

        for zone in self.graph.zones.values():

            x = (
                MARGIN
                + (zone.x - min_x) * scale
            )

            y = (
                MARGIN
                + (zone.y - min_y) * scale
            )

            positions[zone.name] = (x, y)

        return positions

    # ---------------------------------------------------------
    # Draw
    # ---------------------------------------------------------

    def on_draw(self) -> None:

        self.clear()

        self._draw_connections()
        self._draw_zones()
        self._draw_drones()
        self._draw_panel()

    def _draw_connections(self) -> None:

        drawn: set[tuple[str, str]] = set()

        for zone_name, connections in self.graph.connections.items():

            for conn in connections:

                key = conn.key()

                if key in drawn:
                    continue

                drawn.add(key)

                x1, y1 = self.zone_positions[
                    conn.zone_a.name
                ]

                x2, y2 = self.zone_positions[
                    conn.zone_b.name
                ]

                arcade.draw_line(
                    x1,
                    y1,
                    x2,
                    y2,
                    arcade.color.LIGHT_GRAY,
                    2,
                )

    def _draw_zones(self) -> None:

        for zone in self.graph.zones.values():

            x, y = self.zone_positions[zone.name]

            color = COLOR_MAP.get(
                (zone.color or "").lower(),
                arcade.color.WHITE,
            )

            if zone.zone_type == ZoneType.BLOCKED:
                color = arcade.color.DARK_GRAY

            arcade.draw_circle_filled(
                x,
                y,
                NODE_RADIUS,
                color,
            )

            arcade.draw_circle_outline(
                x,
                y,
                NODE_RADIUS,
                arcade.color.WHITE,
                2,
            )

            arcade.draw_text(
                zone.name,
                x - 25,
                y + 25,
                arcade.color.WHITE,
                12,
            )

            arcade.draw_text(
                f"{zone.max_drones}",
                x - 5,
                y - 6,
                arcade.color.BLACK,
                10,
                bold=True,
            )

    def _draw_drones(self) -> None:

        for drone in self.drones:

            if drone.delivered:
                continue

            arcade.draw_circle_filled(
                drone.x,
                drone.y,
                DRONE_RADIUS,
                arcade.color.CYAN,
            )

            arcade.draw_text(
                f"D{drone.drone_id}",
                drone.x + 10,
                drone.y + 10,
                arcade.color.CYAN,
                10,
            )

    def _draw_panel(self) -> None:

        delivered = sum(
            1 for d in self.drones if d.delivered
        )

        text = [
            f"Turn: {self.current_turn}",
            f"Delivered: {delivered}/{self.nb_drones}",
            f"Max Turn: {self.max_turn}",
            "",
            "SPACE = Pause",
            "R = Restart",
        ]

        y = SCREEN_HEIGHT - 30

        for line in text:

            arcade.draw_text(
                line,
                SCREEN_WIDTH - 250,
                y,
                arcade.color.WHITE,
                14,
            )

            y -= 25

    # ---------------------------------------------------------
    # Simulation
    # ---------------------------------------------------------
    def on_update(self, delta_time: float) -> None:

        if self.paused:
            return

        # si todo está terminado → congelar simulación
        if all(d.delivered for d in self.drones):
            self.paused = True
            return

        self.elapsed += delta_time

        progress = min(self.elapsed / TURN_DURATION, 1.0)

        for drone in self.drones:
            drone.x = drone.start_x + (drone.target_x - drone.start_x) * progress
            drone.y = drone.start_y + (drone.target_y - drone.start_y) * progress

        if self.elapsed >= TURN_DURATION:

            self.elapsed = 0.0
            self.current_turn += 1

            self._advance_turn()

    def _advance_turn(self) -> None:

        for drone in self.drones:

            if drone.pending_delivery:
                drone.delivered = True
                drone.pending_delivery = False
                continue

        for drone in self.drones:

            if drone.delivered:
                continue

            next_index = drone.current_step + 1

            if next_index >= len(drone.path):
                drone.delivered = True
                continue

            zone_name, turn = drone.path[next_index]

            if turn != self.current_turn:
                continue

            drone.current_step = next_index

            drone.start_x = drone.x
            drone.start_y = drone.y

            tx, ty = self.zone_positions[
                zone_name
            ]

            drone.target_x = tx
            drone.target_y = ty

            if (
                self.graph.end_zone
                and zone_name == self.graph.end_zone.name
                ):
                    drone.pending_delivery = True

    # ---------------------------------------------------------
    # Controls
    # ---------------------------------------------------------

    def on_key_press(
        self,
        symbol: int,
        modifiers: int,
    ) -> None:

        if symbol == arcade.key.SPACE:
            self.paused = not self.paused

        elif symbol == arcade.key.R:
            self.current_turn = 0
            self.elapsed = 0.0

            for drone in self.drones:

                drone.current_step = 0
                drone.delivered = False

                zone_name = drone.path[0][0]

                x, y = self.zone_positions[
                    zone_name
                ]

                drone.x = x
                drone.y = y

                drone.start_x = x
                drone.start_y = y

                drone.target_x = x
                drone.target_y = y
