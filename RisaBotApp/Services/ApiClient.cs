using System.Net.Http.Json;
using System.Text.Json;
using RisaBotApp.Models;

namespace RisaBotApp.Services;

/// <summary>
/// Centralised HTTP client for all RISA-Bot backend REST calls.
/// Supports port 8000 (WiFi provisioning portal) and port 8080 (ROS 2 dashboard node data & camera feeds).
/// </summary>
public class ApiClient
{
    // ── Defaults ─────────────────────────────────────────────────────────────
    private const string DefaultHotspotUrl  = "http://192.168.4.1:8000";
    private const int    DiscoveryTimeoutMs = 5_000;
    private const int    RosDataTimeoutMs   = 1_500;   // fast timeout for 500ms live polling
    private const int    ScanTimeoutMs      = 30_000;
    private const int    ConnectTimeoutMs   = 60_000;
    private const int    DefaultTimeoutMs   = 10_000;

    private readonly HttpClient _http;
    private string _baseUrl = DefaultHotspotUrl;

    private static readonly JsonSerializerOptions _json = new()
    {
        PropertyNameCaseInsensitive = true
    };

    public ApiClient(HttpClient httpClient)
    {
        _http = httpClient;
    }

    // ── Base URL management ───────────────────────────────────────────────────

    public void UpdateBaseUrl(string url)
    {
        _baseUrl = url.TrimEnd('/');
    }

    public string CurrentBaseUrl => _baseUrl;

    /// <summary>
    /// Derived base URL targeting the ROS 2 dashboard server on port 8080
    /// (e.g. "http://risabot10.local:8080").
    /// </summary>
    public string Port8080BaseUrl
    {
        get
        {
            try
            {
                var uri = new Uri(_baseUrl);
                return $"{uri.Scheme}://{uri.Host}:8080";
            }
            catch
            {
                return "http://192.168.4.1:8080";
            }
        }
    }

    /// <summary>
    /// Base URL for the go2rtc stream server on port 1984.
    /// </summary>
    public string Go2rtcBaseUrl
    {
        get
        {
            try
            {
                var uri = new Uri(_baseUrl);
                return $"{uri.Scheme}://{uri.Host}:1984";
            }
            catch
            {
                return "http://192.168.4.1:1984";
            }
        }
    }

    /// <summary>
    /// MJPEG stream URL served by go2rtc on port 1984.
    /// Single stream slot (astra_camera) — URL never changes between view switches.
    /// View switching is handled server-side by ros2go2rtc_bridge via ROS parameter.
    /// </summary>
    public string CameraStreamUrl => $"{Go2rtcBaseUrl}/stream.html?src=astra_camera&mode=mjpeg";

    // ── Port 8080 ROS 2 Live Endpoints ────────────────────────────────────────

    /// <summary>GET http://<robot_host>:8080/data — live ROS 2 telemetry data.</summary>
    public async Task<ApiResult<RosDashboardData>> GetRosDashboardDataAsync(CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(RosDataTimeoutMs);
        var url = $"{Port8080BaseUrl}/data";
        return await DirectGetAsync<RosDashboardData>(url, cts.Token);
    }

    /// <summary>GET http://<robot_host>:8080/api/set_cam_view?view={view} — switch active camera node processing view.</summary>
    public async Task<ApiResult<bool>> SetCameraViewAsync(string view, CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(DefaultTimeoutMs);
        var url = $"{Port8080BaseUrl}/api/set_cam_view?view={view}";
        try
        {
            var res = await _http.GetAsync(url, cts.Token);
            return res.IsSuccessStatusCode ? ApiResult<bool>.Success(true) : ApiResult<bool>.Failure("Failed to set view");
        }
        catch (Exception ex)
        {
            return ApiResult<bool>.Failure(ex.Message);
        }
    }

    /// <summary>POST http://<robot_host>:8080/api/reset_odom — reset odometry distance & position counters.</summary>
    public async Task<ApiResult<bool>> ResetOdometryAsync(CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(DefaultTimeoutMs);
        var url = $"{Port8080BaseUrl}/api/reset_odom";
        return await DirectPostAsync<object, bool>(url, new { }, cts.Token);
    }

    /// <summary>POST http://<robot_host>:8080/api/reset_competition — send competition command (RESET, LAP1, LAP2).</summary>
    public async Task<ApiResult<bool>> SendCompetitionCommandAsync(string command, CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(DefaultTimeoutMs);
        var url = $"{Port8080BaseUrl}/api/reset_competition";
        return await DirectPostAsync<object, bool>(url, new { command = command }, cts.Token);
    }

    // ── Port 8000 Provisioning / Portal Endpoints ─────────────────────────────

    public async Task<ApiResult<RobotInfo>> GetRobotInfoAsync(CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(DiscoveryTimeoutMs);
        return await GetAsync<RobotInfo>("/api/robot_info", cts.Token);
    }

    public async Task<ApiResult<StatusResponse>> GetStatusAsync(CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(DefaultTimeoutMs);
        return await GetAsync<StatusResponse>("/api/status", cts.Token);
    }

    public async Task<ApiResult<ScanResponse>> ScanNetworksAsync(CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(ScanTimeoutMs);
        return await GetAsync<ScanResponse>("/api/scan", cts.Token);
    }

    public async Task<ApiResult<ConnectResponse>> ConnectAsync(string ssid, string password, CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(ConnectTimeoutMs);
        var body = new ConnectRequest { Ssid = ssid, Password = password };
        return await PostAsync<ConnectRequest, ConnectResponse>("/api/connect", body, cts.Token);
    }

    public async Task<ApiResult<LogsResponse>> GetLogsAsync(CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(DefaultTimeoutMs);
        return await GetAsync<LogsResponse>("/api/logs", cts.Token);
    }

    public async Task<ApiResult<LaunchStatusResponse>> GetLaunchStatusAsync(CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(DefaultTimeoutMs);
        return await GetAsync<LaunchStatusResponse>("/api/launch_status", cts.Token);
    }

    public async Task<ApiResult<LaunchActionResponse>> StartLaunchAsync(CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(DefaultTimeoutMs);
        return await PostAsync<object, LaunchActionResponse>("/api/launch_start", new { }, cts.Token);
    }

    public async Task<ApiResult<LaunchActionResponse>> StopLaunchAsync(CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(DefaultTimeoutMs);
        return await PostAsync<object, LaunchActionResponse>("/api/launch_stop", new { }, cts.Token);
    }

    public async Task<ApiResult<RestartRosResponse>> RestartRosAsync(CancellationToken ct = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        cts.CancelAfter(DefaultTimeoutMs);
        return await PostAsync<object, RestartRosResponse>("/api/restart_ros", new { }, cts.Token);
    }

    // ── Private Helpers ───────────────────────────────────────────────────────

    private string BuildUrl(string path) =>
        _baseUrl.TrimEnd('/') + "/" + path.TrimStart('/');

    private async Task<ApiResult<T>> GetAsync<T>(string path, CancellationToken ct) =>
        await DirectGetAsync<T>(BuildUrl(path), ct);

    private async Task<ApiResult<TResponse>> PostAsync<TRequest, TResponse>(string path, TRequest body, CancellationToken ct) =>
        await DirectPostAsync<TRequest, TResponse>(BuildUrl(path), body, ct);

    private async Task<ApiResult<T>> DirectGetAsync<T>(string fullUrl, CancellationToken ct)
    {
        try
        {
            var response = await _http.GetAsync(fullUrl, ct);
            response.EnsureSuccessStatusCode();
            var data = await response.Content.ReadFromJsonAsync<T>(_json, ct);
            return ApiResult<T>.Success(data!);
        }
        catch (TaskCanceledException)
        {
            return ApiResult<T>.Failure("Request timed out.");
        }
        catch (HttpRequestException ex)
        {
            return ApiResult<T>.Failure($"Network error: {ex.Message}");
        }
        catch (Exception ex)
        {
            return ApiResult<T>.Failure($"Unexpected error: {ex.Message}");
        }
    }

    private async Task<ApiResult<TResponse>> DirectPostAsync<TRequest, TResponse>(string fullUrl, TRequest body, CancellationToken ct)
    {
        try
        {
            var response = await _http.PostAsJsonAsync(fullUrl, body, _json, ct);
            response.EnsureSuccessStatusCode();
            if (typeof(TResponse) == typeof(bool))
            {
                return ApiResult<TResponse>.Success((TResponse)(object)true);
            }
            var data = await response.Content.ReadFromJsonAsync<TResponse>(_json, ct);
            return ApiResult<TResponse>.Success(data!);
        }
        catch (TaskCanceledException)
        {
            return ApiResult<TResponse>.Failure("Request timed out.");
        }
        catch (HttpRequestException ex)
        {
            return ApiResult<TResponse>.Failure($"Network error: {ex.Message}");
        }
        catch (Exception ex)
        {
            return ApiResult<TResponse>.Failure($"Unexpected error: {ex.Message}");
        }
    }
}

/// <summary>
/// Wraps an API call result — either a typed value or an error message.
/// </summary>
public class ApiResult<T>
{
    public T?      Data         { get; private init; }
    public string? ErrorMessage { get; private init; }
    public bool    IsSuccess    => ErrorMessage is null;

    public static ApiResult<T> Success(T data)    => new() { Data = data };
    public static ApiResult<T> Failure(string err) => new() { ErrorMessage = err };
}
