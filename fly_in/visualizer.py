from __future__ import annotations

import arcade
import math
from collections import defaultdict

from graph import Graph
from models import ZoneType

SCREEN_WIDTH = 1800
SCREEN_HEIGHT = 800
SCREEN_TITLE = "Drone Routing Analytics Visualizer"

NODE_RADIUS = 24
DRONE_RADIUS = 8
TURN_DURATION = 1.5  # Segundos por turno
MARGIN = 50

COLOR_MAP = {
    "red": arcade.color.FLAME,
    "green": arcade.color.EMERALD,
    "blue": arcade.color.CORNFLOWER_BLUE,
    "yellow": arcade.color.YELLOW,
    "gray": arcade.color.GRAY,
    "grey": arcade.color.GRAY,
    "cyan": arcade.color.CYAN,
    "magenta": arcade.color.MAGENTA,
    "orange": arcade.color.ORANGE_PEEL,
}


class Drone:
    def __init__(self, drone_id: int, path: list[tuple[str, int]]):
        self.drone_id = drone_id
        self.path = sorted(path, key=lambda p: p[1])

        self.x = 0.0
        self.y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0

        self.current_zone = self.path[0][0] if self.path else ""
        self.next_zone = self.current_zone
        self.delivered = False

    def get_zone_at_turn(self, turn: int) -> str:
        """Devuelve en qué zona está el drone en un turno específico."""
        if not self.path:
            return ""

        if turn >= self.path[-1][1]:
            self.delivered = True
            return self.path[-1][0]

        self.delivered = False
        last_zone = self.path[0][0]

        for zone, t in self.path:
            if t > turn:
                break
            last_zone = zone

        return last_zone


class DroneVisualizer(arcade.Window):
    def __init__(self, graph: Graph,
                 paths: list[list[tuple[str, int]]], nb_drones: int):
        super().__init__(SCREEN_WIDTH, SCREEN_HEIGHT,
                         SCREEN_TITLE, resizable=True)
        arcade.set_background_color(arcade.color.CHARCOAL)

        self.graph = graph
        self.nb_drones = nb_drones
        self.paths = paths

        self.current_turn = 0
        self.elapsed_time = 0.0
        self.paused = False

        self.zone_positions = self._compute_layout()
        self.max_turn = (max(max(turn for _, turn in p)
                             for p in paths) if paths else 0)

        self.drones: list[Drone] = []
        for i, path in enumerate(paths, start=1):
            self.drones.append(Drone(i, path))

        self._update_drone_targets()

    # ---------------------------------------------------------
    # Layout Calculation
    # ---------------------------------------------------------
    def _compute_layout(self) -> dict[str, tuple[float, float]]:
        if not self.graph.zones:
            return {}

        xs = [z.x for z in self.graph.zones.values()]
        ys = [z.y for z in self.graph.zones.values()]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        graph_w = max(max_x - min_x, 1)
        graph_h = max(max_y - min_y, 1)

        scale_x = (self.width - 2 * MARGIN) / graph_w
        scale_y = (self.height - 2 * MARGIN - 60) / graph_h
        scale = min(scale_x, scale_y)

        offset_x = (self.width - (graph_w * scale)) / 2
        offset_y = ((self.height - 60) - (graph_h * scale)) / 2 + 90

        positions = {}
        for zone in self.graph.zones.values():
            x = offset_x + (zone.x - min_x) * scale
            y = offset_y + (zone.y - min_y) * scale
            positions[zone.name] = (x, y)

        return positions

    # ---------------------------------------------------------
    # Logic & Update Loop
    # ---------------------------------------------------------
    def on_key_press(self, key, modifiers):
        if key == arcade.key.SPACE:
            self.paused = not self.paused

    def _update_drone_targets(self):
        """Actualiza las posiciones iniciales y finales de cada dron,

        gestionando el tránsito de 2 turnos hacia zonas restringidas.
        """
        for drone in self.drones:
            z_now = drone.get_zone_at_turn(self.current_turn)
            z_next1 = drone.get_zone_at_turn(self.current_turn + 1)
            z_next2 = drone.get_zone_at_turn(self.current_turn + 2)

            drone.current_zone = z_now
            drone.next_zone = z_next1

            # Posiciones absolutas de las zonas en pixeles
            x_now, y_now = self.zone_positions.get(z_now, (0.0, 0.0))
            x_next1, y_next1 = self.zone_positions.get(z_next1, (x_now, y_now))

            # CASO 1: Primer turno del trayecto hacia una zona RESTRICTED
            # sabemos que en T+2 llegará a una zona restringida.
            if (
                z_now == z_next1 and z_next2 != z_now
                and z_next2 in self.graph.zones
            ):
                target_zone = self.graph.zones[z_next2]
                if target_zone.zone_type == ZoneType.RESTRICTED:
                    x_dest, y_dest = self.zone_positions[z_next2]
                    drone.start_x, drone.start_y = x_now, y_now
            # Destino intermedio: Exactamente en el punto medio de la conexión
                    drone.target_x = x_now + (x_dest - x_now) * 0.5
                    drone.target_y = y_now + (y_dest - y_now) * 0.5
                    continue

            # CASO 2: Segundo turno del trayecto hacia una zona RESTRICTED
    # Lógicamente el plan actualiza que se mueve hacia la zona restringida
            if z_now != z_next1 and z_next1 in self.graph.zones:
                target_zone = self.graph.zones[z_next1]
                if target_zone.zone_type == ZoneType.RESTRICTED:
                    drone.start_x = x_now + (x_next1 - x_now) * 0.5
                    drone.start_y = y_now + (y_next1 - y_now) * 0.5
                    drone.target_x, drone.target_y = x_next1, y_next1
                    continue

            # CASO ESTÁNDAR: Movimiento normal de 1 turno (Normal/Priority)
            drone.start_x, drone.start_y = x_now, y_now
            drone.target_x, drone.target_y = x_next1, y_next1

    def on_update(self, delta_time: float):
        if self.paused or self.current_turn >= self.max_turn:
            return

        self.elapsed_time += delta_time
        progress = min(self.elapsed_time / TURN_DURATION, 1.0)

        # Suavizado de la curva de interpolación (Ease-in-out)
        smooth_progress = progress * progress * (3 - 2 * progress)

        for drone in self.drones:
            drone.x = (
                drone.start_x + (drone.target_x - drone.start_x)
                * smooth_progress
                )
            drone.y = (
                drone.start_y + (drone.target_y - drone.start_y)
                * smooth_progress)

        if self.elapsed_time >= TURN_DURATION:
            self.elapsed_time = 0.0
            self.current_turn += 1
            self._update_drone_targets()

    def on_resize(self, width: float, height: float):
        super().on_resize(width, height)
        self.zone_positions = self._compute_layout()
        self._update_drone_targets()

    # ---------------------------------------------------------
    # Drawing
    # ---------------------------------------------------------
    def on_draw(self):
        self.clear()
        self._draw_connections()
        self._draw_zones()
        self._draw_drones()
        self._draw_ui()

    def _draw_connections(self):
        drawn = set()
        for zone_name, connections in self.graph.connections.items():
            for conn in connections:
                key = conn.key()
                if key in drawn:
                    continue
                drawn.add(key)

                x1, y1 = self.zone_positions[conn.zone_a.name]
                x2, y2 = self.zone_positions[conn.zone_b.name]

                # Gris claro con contraste idóneo para el fondo CHARCOAL
                arcade.draw_line(x1, y1, x2, y2, arcade.color.SLATE_GRAY, 2)

    def _draw_zones(self):
        for zone in self.graph.zones.values():
            x, y = self.zone_positions[zone.name]
            color = (COLOR_MAP.get((zone.color or "").lower(),
                                   arcade.color.ASH_GREY))

            if zone.zone_type == ZoneType.BLOCKED:
                color = arcade.color.OUTER_SPACE
            elif zone.zone_type == ZoneType.RESTRICTED:
                color = arcade.color.INDIAN_RED

            arcade.draw_circle_filled(x, y, NODE_RADIUS + 3, color)
            arcade.draw_circle_filled(x, y, NODE_RADIUS, arcade.color.ONYX)

            arcade.draw_text(
                zone.name,
                x, y,
                arcade.color.WHITE,
                12,
                anchor_x="center",
                anchor_y="center",
                bold=True
            )

            capacidad = getattr(zone, 'max_drones', '∞')
            arcade.draw_text(
                f"Cap: {capacidad}",
                x, y - NODE_RADIUS - 18,
                arcade.color.PALE_AQUA,
                11,
                anchor_x="center",
                bold=True
            )

    def _draw_drones(self):
        positions_map = defaultdict(list)
        for drone in self.drones:
            pos_key = (round(drone.x, 1), round(drone.y, 1))
            positions_map[pos_key].append(drone)

        for (px, py), drone_group in positions_map.items():
            count = len(drone_group)

            for i, drone in enumerate(drone_group):
                offset_x, offset_y = 0, 0

                if count > 1:
                    angle = (i / count) * 2 * math.pi
                    spread_radius = DRONE_RADIUS * 1.5 + (count * 0.9)
                    offset_x = math.cos(angle) * spread_radius
                    offset_y = math.sin(angle) * spread_radius

                final_x = px + offset_x
                final_y = py + offset_y

                if drone.delivered:
                    color = arcade.color.SLATE_GRAY
                    text_color = arcade.color.GRAY
                else:
                    color = arcade.color.WHITE
                    text_color = arcade.color.WHITE

                arcade.draw_circle_filled(final_x, final_y,
                                          DRONE_RADIUS, color)
                arcade.draw_circle_outline(final_x, final_y, DRONE_RADIUS + 1,
                                           arcade.color.BLACK, 2)

                arcade.draw_text(
                    f"D{drone.drone_id}",
                    final_x, final_y + DRONE_RADIUS + 4,
                    text_color,
                    10,
                    anchor_x="center",
                    bold=True
                )

    def _draw_ui(self):
        arcade.draw_rect_filled(
            arcade.XYWH(self.width / 2, 35, self.width, 70),
            arcade.color.ONYX
        )
        arcade.draw_line(0, 70, self.width, 70, arcade.color.ARSENIC, 3)

        status_color = (arcade.color.AERO_BLUE if self.current_turn
                        >= self.max_turn else arcade.color.WHITE)
        status = ("FINALIZADO" if self.current_turn >= self.max_turn
                  else ("PAUSADO" if self.paused else "REPRODUCIENDO"))
        delivered_count = sum(1 for d in self.drones if d.delivered)

        ui_text = f"TURNO: {self.current_turn} / {self.max_turn}   "
        f"|  ENTREGADOS: {delivered_count}/{self.nb_drones} | ESTADO: {status}"

        arcade.draw_text(
            ui_text,
            30, 35,
            status_color,
            14,
            bold=True,
            anchor_y="center"
        )

        arcade.draw_text(
            "[ ESPACIO ] Play / Pause",
            self.width - 30, 35,
            arcade.color.LIGHT_GRAY,
            12,
            anchor_x="right",
            anchor_y="center",
            bold=True
        )
