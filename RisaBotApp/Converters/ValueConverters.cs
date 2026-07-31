using System.Globalization;
using RisaBotApp.Models;

namespace RisaBotApp.Converters;

/// <summary>Inverts a boolean — used to show/hide elements based on negated condition.</summary>
public class InvertBoolConverter : IValueConverter
{
    public object Convert(object? value, Type targetType, object? parameter, CultureInfo culture)
        => value is bool b && !b;
    public object ConvertBack(object? value, Type targetType, object? parameter, CultureInfo culture)
        => value is bool b && !b;
}

/// <summary>
/// Returns one of two colors from a comma-separated ConverterParameter "true_color,false_color".
/// E.g. ConverterParameter="#00C853,#FF5252"
/// </summary>
public class BoolToColorConverter : IValueConverter
{
    public object Convert(object? value, Type targetType, object? parameter, CultureInfo culture)
    {
        var isTrue = value is bool b && b;
        var parts  = (parameter as string)?.Split(',') ?? [];
        var hex    = isTrue
            ? (parts.Length > 0 ? parts[0] : "#FFFFFF")
            : (parts.Length > 1 ? parts[1] : "#888888");
        return Color.FromArgb(hex);
    }
    public object ConvertBack(object? value, Type targetType, object? parameter, CultureInfo culture)
        => throw new NotImplementedException();
}

/// <summary>ROS service active → badge color (green/dim).</summary>
public class BoolToBadgeColorConverter : IValueConverter
{
    public object Convert(object? value, Type targetType, object? parameter, CultureInfo culture)
        => value is true ? Color.FromArgb("#00C853") : Color.FromArgb("#334466");
    public object ConvertBack(object? value, Type targetType, object? parameter, CultureInfo culture)
        => throw new NotImplementedException();
}

/// <summary>True → "ROS Active", False → "ROS Inactive".</summary>
public class BoolToRosStatusConverter : IValueConverter
{
    public object Convert(object? value, Type targetType, object? parameter, CultureInfo culture)
        => value is true ? "ROS Active" : "ROS Inactive";
    public object ConvertBack(object? value, Type targetType, object? parameter, CultureInfo culture)
        => throw new NotImplementedException();
}

/// <summary>True → "Online", False → "Offline".</summary>
public class BoolToOnlineConverter : IValueConverter
{
    public object Convert(object? value, Type targetType, object? parameter, CultureInfo culture)
        => value is true ? "Online" : "Offline";
    public object ConvertBack(object? value, Type targetType, object? parameter, CultureInfo culture)
        => throw new NotImplementedException();
}

/// <summary>
/// Parses a hex colour string (e.g. "#00E676") into a Color, for models that expose
/// their colour as a string. Falls back to transparent on null/garbage rather than throwing.
/// </summary>
public class StringToColorConverter : IValueConverter
{
    public object Convert(object? value, Type targetType, object? parameter, CultureInfo culture)
    {
        if (value is not string hex || string.IsNullOrWhiteSpace(hex)) return Colors.Transparent;
        try { return Color.FromArgb(hex); }
        catch { return Colors.Transparent; }
    }
    public object ConvertBack(object? value, Type targetType, object? parameter, CultureInfo culture)
        => throw new NotImplementedException();
}

/// <summary>Returns true if a string is not null/empty — for IsVisible bindings.</summary>
public class StringNotNullConverter : IValueConverter
{
    public object Convert(object? value, Type targetType, object? parameter, CultureInfo culture)
        => !string.IsNullOrEmpty(value as string);
    public object ConvertBack(object? value, Type targetType, object? parameter, CultureInfo culture)
        => throw new NotImplementedException();
}
