package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Swerve.SwerveDriveConstants;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

/**
 * Follower that reads trajectory CSV exported by the Python generator:
 * CSV format expected: t,x,y,theta
 *   - t: seconds
 *   - x,y: inches
 *   - theta: radians
 *
 * The follower linearly interpolates state at runtime and computes velocities.
 * It converts linear velocities from inches/sec -> meters/sec (IN_TO_M).
 * It converts world velocities to robot-body velocities using imu yaw.
 * Then it computes normalized translation/strafe/rotation inputs for your existing SwerveDrive.drive(...)
 *
 * Drop the CSV at: /sdcard/FIRST/trajectory_out.csv (default) or change TRAJECTORY_PATH.
 */
@Autonomous(name="SwerveTrajectoryFollower", group="Traj")
public class SwerveTrajectoryFollower extends LinearOpMode {

    private static final String TRAJECTORY_PATH = "/sdcard/FIRST/trajectory_out.csv";
    private static final double IN_TO_M = 0.0254;
    private static final double SPEED_SCALE = 1.0; // safety scalar; set <1 for testing

    private SwerveDrive swerveDrive;
    private BHI260IMU imu; // your SwerveDrive uses this IMU type

    private List<TrajectoryPoint> traj = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status","Initializing SwerveTrajectoryFollower...");
        telemetry.update();

        // instantiate swerve drive (uses your constructor that takes CommandOpMode previously)
        // We don't have a CommandOpMode here; your SwerveDrive expects CommandOpMode in constructor.
        // If your SwerveDrive requires CommandOpMode, create a small adapter or add an alternate constructor.
        // Here we assume you have a constructor accepting LinearOpMode or provide a SwerveDrive(LinearOpMode) option.
        // If not, modify SwerveDrive to accept OpMode context or hardwareMap directly.
        try {
            swerveDrive = new SwerveDrive(this); // if SwerveDrive(CommandOpMode) only, create a simple wrapper constructor in SwerveDrive
        } catch (Exception e) {
            telemetry.addLine("Failed to construct SwerveDrive. Ensure SwerveDrive has a constructor accepting OpMode.");
            telemetry.addData("Exception", e.toString());
            telemetry.update();
            waitForStart();
            return;
        }

        // get imu directly if you want; SwerveDrive probably has it but we grab reference if public
        // Attempt to get imu from hardwareMap if SwerveDrive doesn't expose it
        try {
            imu = hardwareMap.get(BHI260IMU.class, "imu");
        } catch (Exception ignored) { imu = null; }

        // load CSV
        boolean ok = loadCsv(TRAJECTORY_PATH);
        if (!ok) {
            telemetry.addData("Error","Failed to load trajectory CSV at: " + TRAJECTORY_PATH);
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addData("Trajectory points", traj.size());
        telemetry.addLine("Press PLAY to start trajectory");
        telemetry.update();

        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        double startTime = runtime.seconds();

        double lastTx = traj.get(0).x;
        double lastTy = traj.get(0).y;
        double lastTth = traj.get(0).th;
        double lastTime = traj.get(0).t;

        // main loop: run until trajectory ends or opmode stops
        while (opModeIsActive()) {
            double now = runtime.seconds() - startTime; // seconds since start
            // if past end -> break and stop robot
            if (now >= traj.get(traj.size()-1).t) {
                telemetry.addLine("Trajectory complete - stopping.");
                // zero drive
                swerveDrive.drive(0,0,0,false);
                telemetry.update();
                break;
            }

            // interpolate state at time now
            TrajectoryPoint cur = sampleTrajectory(now);
            // compute finite-difference velocities using small dt (guard from indexing)
            // find neighboring points for numerical derivative
            double delta = 0.02; // 20ms derivative window
            TrajectoryPoint before = sampleTrajectory(Math.max(0.0, now - delta));
            TrajectoryPoint after  = sampleTrajectory(now + delta);

            double vx_in_per_s = (after.x - before.x) / ( (after.t - before.t) ); // inches/sec
            double vy_in_per_s = (after.y - before.y) / ( (after.t - before.t) );
            double omega_rad_s = (TrajectoryPoint.shortestAngleDiff(before.th, after.th)) / (after.t - before.t); // rad/sec

            // Convert linear velocities to meters/sec for later normalization (SwerveDriveConstants in meters)
            double vx_mps = vx_in_per_s * IN_TO_M;
            double vy_mps = vy_in_per_s * IN_TO_M;

            // Convert world velocities to robot-body velocities using IMU heading (assuming imu gives yaw in radians)
            double imuYaw = 0.0;
            try {
                if (imu != null) {
                    imuYaw = imu.getRobotYawPitchRollAngles().getYaw(); // radians
                } else {
                    // fallback: if SwerveDrive exposes heading getter use it (if available). Otherwise assume field-aligned trajectory and robot heading 0.
                    imuYaw = swerveDrive.getHeading().getRadians();
                }
            } catch (Exception e) {
                // fallback 0
                imuYaw = 0.0;
            }

            // world -> body: vx_body = cos(heading)*vx_world + sin(heading)*vy_world
            double cosh = Math.cos(imuYaw);
            double sinh = Math.sin(imuYaw);
            double vx_body_mps =  cosh * vx_mps + sinh * vy_mps;
            double vy_body_mps = -sinh * vx_mps + cosh * vy_mps;

            // normalize to the inputs expected by your SwerveDrive.drive(translation, strafe, rotation, fieldRelative)
            // Your SwerveDrive multiplies inputs as:
            // new_translation = -translation * SwerveDriveConstants.maxSpeedMeters;
            // new_strafe = -strafe * SwerveDriveConstants.maxSpeedMeters;
            // new_rotation = rotation * maxRadiansPerSecond * rotationMultiplier * 1.5;
            double maxSpeed = SwerveDriveConstants.maxSpeedMeters;
            double maxAng = SwerveDriveConstants.maxRadiansPerSecond;
            double rotMult = SwerveDriveConstants.rotationMultiplier;

            // Apply safety SPEED_SCALE
            vx_body_mps *= SPEED_SCALE;
            vy_body_mps *= SPEED_SCALE;
            omega_rad_s *= SPEED_SCALE;

            double translation_in = - (vx_body_mps / Math.max(1e-6, maxSpeed));  // matches your code sign
            double strafe_in      = - (vy_body_mps / Math.max(1e-6, maxSpeed));
            double rotation_in    = (omega_rad_s / (Math.max(1e-6, maxAng) * rotMult * 1.5));

            // clip to [-1,1]
            translation_in = clamp(translation_in, -1.0, 1.0);
            strafe_in      = clamp(strafe_in, -1.0, 1.0);
            rotation_in    = clamp(rotation_in, -1.0, 1.0);

            // command the drive (fieldRelative=false because we already converted to body frame)
            swerveDrive.drive(translation_in, strafe_in, rotation_in, false);

            // telemetry
            telemetry.addData("t", "%.3f / %.3f", now, traj.get(traj.size()-1).t);
            telemetry.addData("pos (in)", "%.2f, %.2f", cur.x, cur.y);
            telemetry.addData("vel (in/s)", "%.2f, %.2f", vx_in_per_s, vy_in_per_s);
            telemetry.addData("vel (m/s body)", "%.3f, %.3f", vx_body_mps, vy_body_mps);
            telemetry.addData("omega (rad/s)", "%.3f", omega_rad_s);
            telemetry.addData("inputs (tx,st,rot)", "%.3f, %.3f, %.3f", translation_in, strafe_in, rotation_in);
            telemetry.update();

            // small sleep to keep loop predictable - hardware loop is at DT ~ 0.02-0.05
            sleep(20); // ms
        }

        // final stop and exit
        swerveDrive.drive(0,0,0,false);
        telemetry.addLine("Done.");
        telemetry.update();
        idle();
    }

    private boolean loadCsv(String path) {
        File f = new File(path);
        if (!f.exists()) return false;
        List<TrajectoryPoint> list = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(f))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.length() == 0) continue;
                // support optional header? skip if non-numeric first token
                String[] parts = line.split("[,\\s]+");
                if (parts.length < 4) continue;
                try {
                    double t = Double.parseDouble(parts[0]);
                    double x = Double.parseDouble(parts[1]);
                    double y = Double.parseDouble(parts[2]);
                    double th = Double.parseDouble(parts[3]);
                    list.add(new TrajectoryPoint(t, x, y, th));
                } catch (NumberFormatException nfe) {
                    // skip header or invalid line
                    continue;
                }
            }
        } catch (Exception e) {
            telemetry.addData("CSV load error", e.toString());
            telemetry.update();
            return false;
        }
        if (list.isEmpty()) return false;
        traj = list;
        return true;
    }

    /** linear sample; clamps to first/last for out-of-range */
    private TrajectoryPoint sampleTrajectory(double t) {
        if (t <= traj.get(0).t) return traj.get(0);
        if (t >= traj.get(traj.size()-1).t) return traj.get(traj.size()-1);
        // binary search for segment
        int lo = 0, hi = traj.size()-1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (traj.get(mid).t <= t) lo = mid; else hi = mid;
        }
        TrajectoryPoint a = traj.get(lo);
        TrajectoryPoint b = traj.get(hi);
        double u = (t - a.t) / (b.t - a.t);
        return TrajectoryPoint.interp(a, b, u);
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
