using System.Text.Json.Serialization;

namespace RisaBotApp.Models;

/// <summary>
/// WebSocket message received over WS /ws/log during and after a WiFi connect attempt.
/// </summary>
public class WsLogMessage
{
    /// <summary>"step" during connection, "log" for general log lines, "pong" for keep-alive.</summary>
    [JsonPropertyName("type")]
    public string Type { get; set; } = string.Empty;

    /// <summary>
    /// Step status values (in order): connecting | verifying_internet | checking_ros |
    /// checking_hardware | ready | failed
    /// </summary>
    [JsonPropertyName("status")]
    public string Status { get; set; } = string.Empty;

    [JsonPropertyName("message")]
    public string Message { get; set; } = string.Empty;

    /// <summary>Set on "ready" status — wlan0 IP assigned (used internally, not shown in UI).</summary>
    [JsonPropertyName("wlan0_ip")]
    public string? Wlan0Ip { get; set; }

    /// <summary>Set on "ready" status — mDNS-based dashboard URL to store in Preferences.</summary>
    [JsonPropertyName("dashboard_url")]
    public string? DashboardUrl { get; set; }
}

/// <summary>
/// Ordered list of connection steps for the WifiSetupPage progress view.
/// </summary>
public static class ConnectionSteps
{
    public static readonly string[] Ordered =
    [
        "connecting",
        "verifying_internet",
        "checking_ros",
        "checking_hardware",
        "ready"
    ];

    public static string ToDisplayLabel(string status) => status switch
    {
        "connecting"          => "Connecting to WiFi",
        "verifying_internet"  => "Verifying internet",
        "checking_ros"        => "Checking ROS 2 nodes",
        "checking_hardware"   => "Checking sensors & motors",
        "ready"               => "Ready!",
        "failed"              => "Failed",
        _                     => status
    };
}
