def handle_dump_marker_plot(self, request, response):
    """Service handler that writes a PNG of the most recent marker geometry for offline inspection."""
    with self._msg_lock:
        markers_msg = self.latest_markers_msg
        rb_msg = self.latest_rigidbodies_msg

    if markers_msg is None and rb_msg is None:
        response.success = False
        response.message = "No marker/rigidbody data received yet."
        return response

    fig, ax = plt.subplots(figsize=(6, 6))

    # Plot all markers
    if markers_msg:
        xs = [m.translation.x for m in markers_msg.markers]
        ys = [m.translation.y for m in markers_msg.markers]
        ax.scatter(xs, ys, marker='x', label='all markers')
        for m in markers_msg.markers:
            ax.text(m.translation.x, m.translation.y, f"{m.marker_index}", fontsize=8)

    # Plot robot corners and heading
    if rb_msg:
        # try to plot the robot corners of body '1'
        robot_body = next((rb for rb in rb_msg.rigidbodies if rb.rigid_body_name == '1'), None)
        if robot_body:
            corner_pos = []
            for i in [0, 2, 3, 4]:
                corner = next((pt for pt in robot_body.markers if pt.marker_index == i), None)
                if corner:
                    corner_pos.append((corner.translation.x, corner.translation.y))
            if corner_pos:
                cp = np.array(corner_pos)
                ax.scatter(cp[:, 0], cp[:, 1], marker='o', label='robot corners', color='green')
                for idx, (xx, yy) in enumerate(cp):
                    ax.text(xx, yy, f"corner_{idx}", fontsize=8, color='green')

                # Compute robot center
                x_center = np.mean(cp[:, 0])
                y_center = np.mean(cp[:, 1])

                # Front of robot using markers 2 and 3 (correct indices)
                x_front = (cp[1, 0] + cp[2, 0]) / 2.0
                y_front = (cp[1, 1] + cp[2, 1]) / 2.0

                # Heading vector
                dx = x_front - x_center
                dy = y_front - y_center
                ax.arrow(x_center, y_center, dx, dy, head_width=0.02, head_length=0.03, fc='blue', ec='blue', label='heading')

                # Draw line to goal if goal exists
                goal = next((pt for pt in markers_msg.markers if pt.marker_index == int(self.goal_marker_index)), None)
                if goal:
                    x_goal = goal.translation.x
                    y_goal = goal.translation.y
                    ax.plot([x_front, x_goal], [y_front, y_goal], 'r--', label='robot-to-goal')

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Latest Markers and Robot Corners (index labels)')
    ax.legend()
    ax.axis('equal')
    # Save file
    outpath = '/tmp/controller_marker_dump.png'
    try:
        fig.savefig(outpath)
        plt.close(fig)
        response.success = True
        response.message = f"Saved marker plot to {outpath}"
    except Exception as e:
        response.success = False
        response.message = f"Failed to save plot: {e}"
    return response
