package org.firstinspires.ftc.teamcode.drive.LaserDrive

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.hardware.sparkfun.SparkFunOTOS

/**
 * Abstraction for generic robot drive motion and localization. Robot poses are specified in a coordinate system with
 * positive x pointing forward, positive y pointing left, and positive heading measured counter-clockwise from the
 * x-axis.
 */
abstract class Drive {
    /**
     * Localizer used to determine the evolution of [poseEstimate].
     */
    abstract var localizer: Localizer

    private var headingOffset: Double = 0.0

    /**
     * The raw heading used for computing [externalHeading]. Not affected by [externalHeading] setter.
     */
    protected abstract val rawExternalHeading: Double

    /**
     * The robot's heading in radians as measured by an external sensor (e.g., IMU, gyroscope).
     */
    var externalHeading: Double
        get() = Angle.norm(rawExternalHeading + headingOffset)
        set(value) {
            headingOffset = -rawExternalHeading + value
        }

    /**
     * SparkFun OTOS optical tracking sensor reference.
     * Should be initialized externally before use.
     */
    lateinit var optical: SparkFunOTOS

    // Backing field for poseEstimate
    private var _poseEstimate: Pose2d = Pose2d()

    /**
     * The robot's current pose estimate.
     * Returns the OTOS position if available; otherwise returns the last known estimate.
     */
    var poseEstimate: Pose2d
        get() {
            if (this::optical.isInitialized) {
                val opticalPose = optical.position
                _poseEstimate = Pose2d(opticalPose.x, opticalPose.y, opticalPose.h)
            }
            return _poseEstimate
        }
        set(value) {
            _poseEstimate = value
        }

    /**
     * Current robot pose velocity (optional)
     */
    val poseVelocity: Pose2d?
        get() = localizer.poseVelocity

    /**
     * Updates [poseEstimate] with the most recent positional change.
     */
    protected var pose: Pose2d = Pose2d()
    private val poseHistory = ArrayDeque<Pose2d>()

    // Replace localizer.update() with your own
    open fun updateOTOSPose(optical: SparkFunOTOS) {
        val opticalPose = optical.position
        pose = Pose2d(opticalPose.x, opticalPose.y, opticalPose.h)

        poseHistory.add(pose)
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        // Optional: log it
        // FlightRecorder.write("ESTIMATED_POSE", PoseMessage(pose))
    }

    /**
     * Returns the current OTOS-based pose estimate.
     */
    open fun getOTOSPoseEstimate(): Pose2d = pose

    /**
     * Sets the current commanded drive state of the robot. Feedforward is applied to [driveSignal] before it reaches
     * the motors.
     */
    abstract fun setDriveSignal(driveSignal: DriveSignal)

    /**
     * Sets the current commanded drive state of the robot. Feedforward is *not* applied to [drivePower].
     */
    abstract fun setDrivePower(drivePower: Pose2d)

    /**
     * The heading velocity used to determine pose velocity in some cases.
     */
    open fun getExternalHeadingVelocity(): Double? = null
}
