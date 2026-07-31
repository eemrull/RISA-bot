using System.Text.Json.Serialization;

namespace RisaBotApp.Models;

/// <summary>Response from GET /api/logs.</summary>
public class LogsResponse
{
    [JsonPropertyName("has_error")]
    public bool HasError { get; set; }

    [JsonPropertyName("error_count")]
    public int ErrorCount { get; set; }

    [JsonPropertyName("errors")]
    public List<string> Errors { get; set; } = [];

    [JsonPropertyName("logs")]
    public List<string> Logs { get; set; } = [];

    [JsonPropertyName("status_summary")]
    public string StatusSummary { get; set; } = string.Empty;
}

/// <summary>Response from GET /api/launch_status.</summary>
public class LaunchStatusResponse
{
    [JsonPropertyName("launch_active")]
    public bool LaunchActive { get; set; }

    [JsonPropertyName("service_state")]
    public string ServiceState { get; set; } = string.Empty;
}

/// <summary>Response from POST /api/launch_start and POST /api/launch_stop.</summary>
public class LaunchActionResponse
{
    [JsonPropertyName("ok")]
    public bool Ok { get; set; }

    [JsonPropertyName("launch_active")]
    public bool LaunchActive { get; set; }

    [JsonPropertyName("service_state")]
    public string ServiceState { get; set; } = string.Empty;

    [JsonPropertyName("output")]
    public string Output { get; set; } = string.Empty;
}

/// <summary>Response from POST /api/restart_ros.</summary>
public class RestartRosResponse
{
    [JsonPropertyName("status")]
    public string Status { get; set; } = string.Empty;

    [JsonPropertyName("output")]
    public string Output { get; set; } = string.Empty;
}

/// <summary>Request body for POST /api/connect.</summary>
public class ConnectRequest
{
    [JsonPropertyName("ssid")]
    public string Ssid { get; set; } = string.Empty;

    [JsonPropertyName("password")]
    public string Password { get; set; } = string.Empty;
}

/// <summary>Response from POST /api/connect.</summary>
public class ConnectResponse
{
    [JsonPropertyName("status")]
    public string Status { get; set; } = string.Empty;

    [JsonPropertyName("ssid")]
    public string Ssid { get; set; } = string.Empty;
}
