using System.Text.Json.Serialization;

namespace RisaBotApp.Models;

/// <summary>
/// Response from GET /api/status.
/// Reflects wlan0 connection state, internet reachability, and ROS 2 service health.
/// </summary>
public class StatusResponse
{
    [JsonPropertyName("wlan0_state")]
    public string Wlan0State { get; set; } = string.Empty;

    [JsonPropertyName("connected_ssid")]
    public string? ConnectedSsid { get; set; }

    [JsonPropertyName("wlan0_ip")]
    public string? Wlan0Ip { get; set; }

    [JsonPropertyName("ap_ip")]
    public string ApIp { get; set; } = string.Empty;

    [JsonPropertyName("internet_online")]
    public bool InternetOnline { get; set; }

    [JsonPropertyName("ros_service_active")]
    public bool RosServiceActive { get; set; }

    [JsonPropertyName("launch_active")]
    public bool LaunchActive { get; set; }

    /// <summary>Preferred hostname-based dashboard URL (e.g. http://risabot9.local/dashboard).</summary>
    [JsonPropertyName("dashboard_url")]
    public string DashboardUrl { get; set; } = string.Empty;

    [JsonPropertyName("hostname_dashboard_url")]
    public string HostnameDashboardUrl { get; set; } = string.Empty;

    [JsonPropertyName("web_dashboard_url")]
    public string WebDashboardUrl { get; set; } = string.Empty;

    [JsonPropertyName("wlan0_dashboard_url")]
    public string Wlan0DashboardUrl { get; set; } = string.Empty;
}
