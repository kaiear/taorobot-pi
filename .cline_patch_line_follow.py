from pathlib import Path
p = Path.cwd() / 'src' / 'tao_tasks' / 'scripts' / 'line_follow_controller.py'
s = p.read_text(encoding='utf-8')
repls = [
('''        self.fixed_pick_sequence_deg = private_param(
            "fixed_pick_sequence_deg",
            [
                [0, 60, -50, 3, 0, 50],
                [0, 80, -50, -5, 0, 50],
                [0, 80, -50, -5, 0, -30],
                [0, 0, 0, 0, 0, -30],
            ],
        )
        self.move_x = self.arm_err_x
''', '''        self.fixed_pick_sequence_deg = private_param(
            "fixed_pick_sequence_deg",
            [
                [0, 60, -50, 3, 0, 50],
                [0, 80, -50, -5, 0, 50],
                [0, 80, -50, -5, 0, -30],
                [0, 0, 0, 0, 0, -30],
            ],
        )
        # Fixed place poses are loaded from YAML, so field-tuned/C++ task
        # strategy remains outside the Python source.
        self.fixed_place_prepose_enabled = as_bool(private_param("fixed_place_prepose_enabled", False))
        self.fixed_place_prepose_duration_ms = int(private_param("fixed_place_prepose_duration_ms", 1000))
        self.fixed_place_prepose_hold_ticks = max(1, int(private_param("fixed_place_prepose_hold_ticks", 20)))
        self.fixed_place_prepose_deg_by_cross = private_param("fixed_place_prepose_deg_by_cross", {})
        self.fixed_place_sequence_enabled = as_bool(private_param("fixed_place_sequence_enabled", False))
        self.fixed_place_sequence_duration_ms = int(private_param("fixed_place_sequence_duration_ms", 1000))
        self.fixed_place_sequence_hold_ticks = max(1, int(private_param("fixed_place_sequence_hold_ticks", 20)))
        self.fixed_place_sequence_deg_by_cross = private_param("fixed_place_sequence_deg_by_cross", {})
        self.move_x = self.arm_err_x
'''),
('''        self.fixed_pick_last_sent_index = -1
        self.last_color_debug_log_time = rospy.Time(0)
''', '''        self.fixed_pick_last_sent_index = -1
        self.fixed_place_prepose_sent = False
        self.fixed_place_prepose_tick = 0
        self.fixed_place_sequence_index = 0
        self.fixed_place_sequence_tick = 0
        self.fixed_place_last_sent_index = -1
        self.last_color_debug_log_time = rospy.Time(0)
'''),
('''                    self.move_y = 80.0
                    self.move_arm(self.move_x, self.move_y, self.arm_up, 1000)
                    self.time_count = 0
                    self.arm_task_phase = ARM_PHASE_PLACE
                    self.move_status = 2
                    self.line_mode = False
''', '''                    self.move_y = 80.0
                    self.time_count = 0
                    self.arm_task_phase = ARM_PHASE_PLACE
                    self.move_status = 2
                    self.fixed_place_prepose_sent = False
                    self.fixed_place_prepose_tick = 0
                    self.fixed_place_sequence_index = 0
                    self.fixed_place_sequence_tick = 0
                    self.fixed_place_last_sent_index = -1
                    if not self.handle_fixed_place_prepose():
                        self.move_arm(self.move_x, self.move_y, self.arm_up, 1000)
                    self.line_mode = False
'''),
('''        if self.arm_task_phase == ARM_PHASE_PLACE and self.move_status == 2:
            self.time_count += 1
''', '''        if self.arm_task_phase == ARM_PHASE_PLACE and self.move_status == 2:
            if self.handle_fixed_place_prepose():
                return cmd, "fixed_place_prepose"
            self.time_count += 1
'''),
('''        if self.arm_task_phase == ARM_PHASE_PLACE and self.move_status == 5:
            self.time_count += 1
''', '''        if self.arm_task_phase == ARM_PHASE_PLACE and self.move_status == 5:
            sequence = self.get_cross_config(self.fixed_place_sequence_deg_by_cross, self.cross_count, [])
            if self.fixed_place_sequence_enabled and sequence:
                return self.handle_fixed_place_sequence(cmd, sequence)
            self.time_count += 1
''')]
for old, new in repls:
    if old not in s:
        raise SystemExit('anchor not found: ' + old.splitlines()[0].strip())
    s = s.replace(old, new, 1)
old = '''        if self.fixed_pick_sequence_tick >= self.fixed_pick_sequence_hold_ticks:
            self.fixed_pick_sequence_index += 1
            self.fixed_pick_sequence_tick = 0
        return cmd, "fixed_pick_sequence"

'''
new = old + '''    @staticmethod
    def get_cross_config(config, cross, default=None):
        if config is None:
            return default
        if isinstance(config, dict):
            for key in (cross, str(cross), float(cross)):
                if key in config:
                    return config[key]
        return default

    def handle_fixed_place_prepose(self):
        joints_deg = self.get_cross_config(self.fixed_place_prepose_deg_by_cross, self.cross_count, None)
        if not self.fixed_place_prepose_enabled or not joints_deg:
            return False
        if not self.fixed_place_prepose_sent:
            self.publish_fixed_arm_degrees(joints_deg, self.fixed_place_prepose_duration_ms, label="fixed_place_prepose cross={}".format(self.cross_count))
            self.fixed_place_prepose_sent = True
            self.fixed_place_prepose_tick = 0
            return True
        if self.fixed_place_prepose_tick < self.fixed_place_prepose_hold_ticks:
            self.fixed_place_prepose_tick += 1
            return True
        return False

    def handle_fixed_place_sequence(self, cmd, sequence):
        self.time_count += 1
        if self.fixed_place_sequence_index >= len(sequence):
            rospy.loginfo("fixed place sequence done: cross=%d", self.cross_count)
            self.finish_place_phase()
            return cmd, "fixed_place_done"
        if self.fixed_place_last_sent_index != self.fixed_place_sequence_index:
            joints_deg = sequence[self.fixed_place_sequence_index]
            self.publish_fixed_arm_degrees(joints_deg, self.fixed_place_sequence_duration_ms, label="fixed_place step={}/{} cross={}".format(self.fixed_place_sequence_index + 1, len(sequence), self.cross_count))
            self.fixed_place_last_sent_index = self.fixed_place_sequence_index
            self.fixed_place_sequence_tick = 0
        self.fixed_place_sequence_tick += 1
        if self.fixed_place_sequence_tick >= self.fixed_place_sequence_hold_ticks:
            self.fixed_place_sequence_index += 1
            self.fixed_place_sequence_tick = 0
        return cmd, "fixed_place_sequence"

'''
if old not in s:
    raise SystemExit('method insert anchor not found')
s = s.replace(old, new, 1)
old = '''        else:
            self.crossing_flag = 1

    def reset_crossing'''
new = '''        else:
            self.crossing_flag = 1

    def finish_place_phase(self):
        self.line_mode = True
        self.crossing_flag = 1
        self.arm_task_phase = ARM_PHASE_IDLE
        self.move_status = 0
        self.captured_color = BLOCK_NONE
        self.time_count = 0
        self.last_color_blob = None
        self.last_color_seen_countdown = 0
        self.move_x = self.arm_err_x
        self.move_y = 140.0
        self.spin_claw = 0.0
        self.fixed_place_prepose_sent = False
        self.fixed_place_prepose_tick = 0
        self.fixed_place_sequence_index = 0
        self.fixed_place_sequence_tick = 0
        self.fixed_place_last_sent_index = -1

    def reset_crossing'''
if old not in s:
    raise SystemExit('finish insert anchor not found')
s = s.replace(old, new, 1)
with open(str(p), 'w', encoding='utf-8', newline='') as f:
    f.write(s)
