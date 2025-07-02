def step(self, action):
        total_reward = 0.0
        max_reward = float('-inf')  # ⬅track worst (smallest) reward seen
        hit_occurred = False
        done = False
        hit_reward = 0.0
        self.frame_skips = getattr(self, "frame_skips", 5)

        for _ in range(self.frame_skips):
            self.ros_world.step(render=True)
            world_move_x = action[0] * 2.0   # x ∈ [-2, 2]
            world_move_y = action[1] * 2.0   # y ∈ [-2, 2]
            world_move_z = (action[2] + 1.0) * 0.45 + 0.3   # z ∈ [0.3, 1.2]
            # self.want_to_hit = bool(action[3] > 0.5)

            world_action = np.array([world_move_x, world_move_y, world_move_z, 1.0])
            robot_action = world_action.copy()

            # self.motion_controller.racket_control.recover_racket()
            self.cmd_pub_point.publish(Point(
                x=float(robot_action[0]),
                y=float(robot_action[1]),
                z=float(robot_action[2])
            ))
            # self.cmd_pub_bool.publish(Bool(data=self.want_to_hit))
            self.motion_controller.TR_status_update()
            if self.motion_controller.body_control.pending_action:
                self.motion_controller.body_control.move_robot()
            # if self.motion_controller.racket_control.pending_action_hit:
            #     print("hit")
            #     self.motion_controller.racket_control.hit_racket()
            #     # contact check
            #     # ball_hit, illegal_hit = self._check_contact_sensor_legality()
            #     # if ball_hit:
            #     #     hit_occurred = True
            #     #     self.successful_hits += 1
            #     #     if illegal_hit:
            #     #         print("Illegal Hit Detected!")
            #     #         hit_reward = -0.5  # punishment for hitting with the wrong link
            #     #     else:
            #     #         print("Legal Hit Detected!")
            #     #         hit_reward = self.ball_velocity[1]  # reward for y+ speed
            #     hit_reward = self.ball_velocity[1]  # reward for y+ speed
            #     hit_occurred = True
            #     self.successful_hits += 1
            if self.motion_controller.racket_control.pending_action_recover:
                self.motion_controller.racket_control.recover_racket()

            obs = self._get_latest_obs()
            self.prev_position = obs[:3].copy()
            reward = self._compute_reward(self.prev_position, obs[:3], obs[3:6])
            max_reward = max(reward,max_reward)
            ball_speed = np.linalg.norm(self.ball_velocity[:3])
            self.total_steps += 1

            out_of_bounds = (self.ball_position[1] < 0) or (self.ball_position[0] < 0) or (self.ball_position[0] > 5)
            too_slow = ball_speed < 0.2
            current_sim_time = self.timeline.get_current_time()
            sim_elapsed = current_sim_time - self.start_sim_time
            too_long = sim_elapsed > 5.0

            if out_of_bounds or too_long or too_slow:
                done = True
                break

        # Final reward logic
        # if hit_occurred:
        #     total_reward = reward + hit_reward
        # else:
        #     total_reward = min_reward  # Use worst reward seen if not hit

        total_reward = max_reward

        total_reward *= 0.1
        total_reward = np.clip(total_reward, -1.0, 1.0)
        print(f"[Step] total_reward:{total_reward}")

        return obs, total_reward, done, False, {}
    
    # def _check_contact_sensor_legality(self):
    #     result = self.contact_sensor.get_current_frame()
    #     print("[DEBUG] Contact sensor result:", result)
    #     if not result["in_contact"]:
    #         return False, False  # no contact, no hit

    #     contacted_body_names = [str(body.GetPath()) for body in result["body1"]]
    #     is_ball_hit = any("tennis_ball" in name for name in contacted_body_names)
    #     is_illegal = not any("Racket_Rev_Link" in name for name in contacted_body_names)

    #     return is_ball_hit, is_illegal

