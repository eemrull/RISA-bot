namespace RisaBotApp.Models;

/// <summary>
/// A single log entry line parsed from journalctl, with error highlighting status.
/// </summary>
public class LogEntry
{
    public string Text    { get; init; } = string.Empty;
    public bool   IsError { get; init; }
}
