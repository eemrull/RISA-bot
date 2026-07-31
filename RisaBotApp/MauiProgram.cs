using Microcharts.Maui;
using Microsoft.Extensions.Logging;
using RisaBotApp.Pages;
using RisaBotApp.Services;
using RisaBotApp.ViewModels;

namespace RisaBotApp;

public static class MauiProgram
{
    public static MauiApp CreateMauiApp()
    {
        var builder = MauiApp.CreateBuilder();

        builder
            .UseMauiApp<App>()
            .UseMicrocharts()
            .ConfigureFonts(fonts =>
            {
                fonts.AddFont("Inter_18pt-Regular.ttf",   "InterRegular");
                fonts.AddFont("Inter_18pt-SemiBold.ttf",  "InterSemiBold");
                fonts.AddFont("Inter_18pt-Bold.ttf",      "InterBold");
            });

        // ── Services (Singletons — shared across all pages) ─────────────────
        builder.Services.AddSingleton<HttpClient>();
        builder.Services.AddSingleton<ApiClient>();
        builder.Services.AddSingleton<RobotConnectionService>();
        builder.Services.AddSingleton<RobotPairingService>();
        builder.Services.AddSingleton<RobotDiscoveryService>();

        // ── ViewModels (Transient — fresh instance per navigation) ──────────
        builder.Services.AddTransient<DiscoveryViewModel>();
        builder.Services.AddTransient<WifiSetupViewModel>();
        builder.Services.AddTransient<DashboardViewModel>();

        // ── Pages (Transient — recreated each navigation) ───────────────────
        builder.Services.AddTransient<SplashPage>();
        builder.Services.AddTransient<DiscoveryPage>();
        builder.Services.AddTransient<WifiSetupPage>();
        builder.Services.AddTransient<DashboardPage>();

#if DEBUG
        builder.Logging.AddDebug();
#endif

        return builder.Build();
    }
}
