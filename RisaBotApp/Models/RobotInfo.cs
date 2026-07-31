using System.Text.Json.Serialization;

namespace RisaBotApp.Models;

/// <summary>
/// Response from GET /api/robot_info.
/// Used by DiscoveryPage to confirm the phone is connected to the correct robot hotspot.
/// </summary>
public class RobotInfo
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = string.Empty;

    [JsonPropertyName("serial")]
    public string Serial { get; set; } = string.Empty;

    [JsonPropertyName("model")]
    public string Model { get; set; } = string.Empty;

    [JsonPropertyName("firmware_version")]
    public string FirmwareVersion { get; set; } = string.Empty;

    /// <summary>Raw hostname reported by the robot (e.g. "risabot9").</summary>
    [JsonPropertyName("hostname")]
    public string Hostname { get; set; } = string.Empty;

    /// <summary>
    /// mDNS hostname (e.g. "risabot9.local").
    /// This is the address used everywhere in the app — raw IPs are never shown.
    /// </summary>
    [JsonPropertyName("mdns_hostname")]
    public string MdnsHostname { get; set; } = string.Empty;
}
