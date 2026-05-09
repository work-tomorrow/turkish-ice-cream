import math

import pygame

from common import WorldConfig, Vec2, move_toward
from state_machine import ArmStateMachine


HAND_MAX_SPEED = 1000.0

BACKGROUND_COLOR = (245, 245, 245)

HAND_COLOR = (255, 120, 120)
HAND_BORDER_COLOR = (180, 50, 50)

HAND_HIDDEN_COLOR = (180, 120, 120)
HAND_HIDDEN_BORDER_COLOR = (100, 70, 70)

TEXT_COLOR = (30, 30, 30)

BLIND_ZONE_COLOR = (220, 200, 245)
BLIND_ZONE_BORDER_COLOR = (140, 90, 180)

CONE_COLOR = (252, 236, 216)
CONE_BORDER_COLOR = (238, 132, 139)


def draw_circle_with_border(
    screen,
    pos: Vec2,
    radius: int,
    fill_color,
    border_color,
    border_width: int = 3,
):
    pygame.draw.circle(
        screen,
        fill_color,
        (int(pos.x), int(pos.y)),
        radius,
    )

    pygame.draw.circle(
        screen,
        border_color,
        (int(pos.x), int(pos.y)),
        radius,
        border_width,
    )


def draw_blind_zone(screen, config: WorldConfig):
    rect = pygame.Rect(
        int(config.blind_zone.x),
        int(config.blind_zone.y),
        int(config.blind_zone.w),
        int(config.blind_zone.h),
    )

    pygame.draw.rect(
        screen,
        BLIND_ZONE_COLOR,
        rect,
        border_radius=8,
    )

    pygame.draw.rect(
        screen,
        BLIND_ZONE_BORDER_COLOR,
        rect,
        width=2,
        border_radius=8,
    )


def draw_cone(
    screen,
    center: Vec2,
    angle: float,
    config: WorldConfig,
):
    outward = Vec2(math.cos(angle), math.sin(angle))
    tangent = Vec2(-outward.y, outward.x)

    base_center = center + outward * config.ice_radius
    half_width = config.cone_base_width * 0.5

    base_a = base_center + tangent * half_width
    base_b = base_center - tangent * half_width
    tip = center + outward * (config.ice_radius + config.cone_length)

    points = [
        (int(base_a.x), int(base_a.y)),
        (int(tip.x), int(tip.y)),
        (int(base_b.x), int(base_b.y)),
    ]

    pygame.draw.polygon(screen, CONE_COLOR, points)
    pygame.draw.polygon(screen, CONE_BORDER_COLOR, points, width=5)


def get_observed_hand_pos(
    true_hand_pos: Vec2 | None,
    config: WorldConfig,
) -> tuple[Vec2 | None, bool]:
    """
    模擬 camera blind zone。

    true_hand_pos:
        pygame 中真正的手位置。

    observed_hand_pos:
        camera 看得到的手位置。

    如果手進入 blind zone，就模擬成 camera 看不到手。
    """

    if true_hand_pos is None:
        return None, False

    if config.blind_zone.contains(true_hand_pos):
        return None, True

    return true_hand_pos, False


def main():
    pygame.init()

    config = WorldConfig()
    camera_period = 1.0 / config.camera_fps

    screen = pygame.display.set_mode((config.width, config.height))
    pygame.display.set_caption("Turkish Ice Cream State Machine Lab")

    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    state_machine = ArmStateMachine(config)

    ice_pos = Vec2(config.width / 2, config.height / 2)
    true_hand_pos = None
    observed_hand_pos = None

    ice_color = (120, 190, 255)
    ice_border_color = (40, 90, 180)
    status = "init"
    camera_lost_by_blind_zone = False
    camera_timer = camera_period
    cone_angle = -math.pi / 2.0

    running = True

    while running:
        dt = clock.tick(config.render_fps) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    ice_pos = Vec2(config.width / 2, config.height / 2)
                    true_hand_pos = None
                    observed_hand_pos = None
                    state_machine = ArmStateMachine(config)
                    camera_timer = camera_period
                    cone_angle = -math.pi / 2.0

        mouse_focused = pygame.mouse.get_focused()

        if mouse_focused:
            mx, my = pygame.mouse.get_pos()
            mouse_pos = Vec2(mx, my)

            if true_hand_pos is None:
                true_hand_pos = mouse_pos
            else:
                true_hand_pos = move_toward(
                    current=true_hand_pos,
                    target=mouse_pos,
                    max_distance=HAND_MAX_SPEED * dt,
                )
        else:
            true_hand_pos = None

        camera_timer += dt
        camera_sample_fresh = False

        if camera_timer >= camera_period:
            observed_hand_pos, camera_lost_by_blind_zone = get_observed_hand_pos(
                true_hand_pos=true_hand_pos,
                config=config,
            )
            camera_timer %= camera_period
            camera_sample_fresh = True

        result = state_machine.update(
            ice_pos=ice_pos,
            hand_pos=observed_hand_pos,
            dt=dt,
            hand_sample_fresh=camera_sample_fresh,
        )

        ice_pos = result.pos
        ice_color = result.color
        ice_border_color = result.border_color
        status = result.status
        if result.cone_angle is not None:
            cone_angle = result.cone_angle

        screen.fill(BACKGROUND_COLOR)

        draw_blind_zone(screen, config)

        if true_hand_pos is not None:
            if observed_hand_pos is None:
                hand_color = HAND_HIDDEN_COLOR
                hand_border_color = HAND_HIDDEN_BORDER_COLOR
            else:
                hand_color = HAND_COLOR
                hand_border_color = HAND_BORDER_COLOR

            draw_circle_with_border(
                screen,
                true_hand_pos,
                int(config.hand_radius),
                hand_color,
                hand_border_color,
                border_width=3,
            )

        draw_cone(
            screen,
            ice_pos,
            cone_angle,
            config,
        )

        draw_circle_with_border(
            screen,
            ice_pos,
            int(config.ice_radius),
            ice_color,
            ice_border_color,
            border_width=3,
        )

        info_lines = [
            f"State: {state_machine.state.name}",
            f"State time: {state_machine.state_time:.2f}s",
            f"Status: {status}",
            f"Reason: {state_machine.transition_reason}",
            f"Camera observed hand: {observed_hand_pos is not None}",
            f"Camera lost by blind zone: {camera_lost_by_blind_zone}",
            f"Camera FPS: {config.camera_fps:.0f}",
            f"Control period: {config.control_period:.2f}s",
            f"Motor speed: {config.motor_nominal_speed:.0f} +/- {config.motor_speed_delta_ratio * 100:.0f}%",
            f"Cone angle: {math.degrees(cone_angle):.0f} deg",
            f"Fresh camera sample: {camera_sample_fresh}",
            "",
            "Red hand = camera can see hand",
            "Dark red hand = true hand exists but camera cannot see it",
            "Blue ice = normal escape",
            "Orange ice = boundary escape",
            "Gray ice = close retreat",
            "Purple ice = blind spot escape",
            "Light purple box = blind zone",
            "Press R to reset",
            f"Ice position: ({ice_pos.x:.1f}, {ice_pos.y:.1f})",
        ]

        y = 12
        for line in info_lines:
            text = font.render(line, True, TEXT_COLOR)
            screen.blit(text, (12, y))
            y += 24

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
