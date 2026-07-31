using System.Text.Json.Serialization;

namespace RisaBotApp.Models;

/// <summary>
/// Model representing the live ROS 2 telemetry data returned by GET /data on port 8080.
/// </summary>
public class RosDashboardData
{
    [JsonPropertyName("state")]
    public string State { get; set; } = "UNKNOWN";

    [JsonPropertyName("lap")]
    public int Lap { get; set; } = 1;

    [JsonPropertyName("auto_mode")]
    public bool AutoMode { get; set; }

    [JsonPropertyName("speed")]
    public double Speed { get; set; }

    [JsonPropertyName("distance")]
    public double Distance { get; set; }

    [JsonPropertyName("odom_x")]
    public double OdomX { get; set; }

    [JsonPropertyName("odom_y")]
    public double OdomY { get; set; }

    [JsonPropertyName("odom_yaw")]
    public double OdomYaw { get; set; }

    [JsonPropertyName("lane_error")]
    public double LaneError { get; set; }

    [JsonPropertyName("imu_roll")]
    public double ImuRoll { get; set; }

    [JsonPropertyName("imu_pitch")]
    public double ImuPitch { get; set; }

    [JsonPropertyName("imu_yaw")]
    public double ImuYaw { get; set; }

    [JsonPropertyName("traffic_light")]
    public string TrafficLight { get; set; } = "unknown";

    [JsonPropertyName("boom_gate")]
    public bool? BoomGate { get; set; }

    [JsonPropertyName("tunnel_detected")]
    public bool? TunnelDetected { get; set; }

    [JsonPropertyName("parking_sign_detected")]
    public bool? ParkingSignDetected { get; set; }

    [JsonPropertyName("health_ok")]
    public bool? HealthOk { get; set; }

    [JsonPropertyName("health_summary")]
    public string HealthSummary { get; set; } = string.Empty;

    [JsonPropertyName("cmd_lin_x")]
    public double CmdLinX { get; set; }

    [JsonPropertyName("cmd_ang_z")]
    public double CmdAngZ { get; set; }
}
