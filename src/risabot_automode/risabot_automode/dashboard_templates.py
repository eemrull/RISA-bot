# dashboard_templates.py
# Auto-extracted HTML templates for the RISA-bot dashboard.
# Edit these to change the dashboard UI appearance.

# ======================== Main HTML Dashboard ========================
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RISA-Bot Dashboard</title>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;800&display=swap" rel="stylesheet">
<style>
  :root {
    /* Catppuccin Latte (Light Theme) */
    --bg: #eff1f5;
    --surface: #e6e9ef;
    --surface2: #ccd0da;
    --text: #4c4f69;
    --accent: #1e66f5; /* Sapphire */
    --danger: #d20f39; /* Red */
    --success: #40a02b; /* Green */
    --warning: #df8e1d; /* Yellow */
    --muted: #8c8fa1;
    --radius: 16px;
    --glow: rgba(30, 102, 245, 0.15);
  }
  * { margin:0; padding:0; box-sizing:border-box; }
  body {
    font-family: 'Inter', system-ui, -apple-system, sans-serif;
    font-size: 18px; /* Bigger base size for readability */
    font-weight: 500;
    background: var(--bg);
    color: var(--text);
    min-height: 100vh;
    overflow-x: hidden;
    position: relative;
  }

  /* ===== ANIMATED ORB BACKGROUND ===== */
  .bg-orbs {
    position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
    z-index: -1; overflow: hidden; pointer-events: none;
  }
  .orb {
    position: absolute; border-radius: 50%; filter: blur(60px);
    animation: float 20s infinite ease-in-out alternate; opacity: 0.15;
  }
  .orb-1 { width: 400px; height: 400px; background: rgba(233,69,96,1); top: -10%; left: 10%; animation-delay: 0s; }
  .orb-2 { width: 500px; height: 500px; background: rgba(15,52,96,1); bottom: -20%; right: -10%; animation-delay: -5s; }
  .orb-3 { width: 300px; height: 300px; background: rgba(66,165,245,1); top: 50%; left: 60%; animation-delay: -10s; }
  @keyframes float { 0% { transform: translateY(0) scale(1); } 100% { transform: translateY(-50px) scale(1.1); } }

  /* ===== WARNING FLASH OVERLAY ===== */
  .warning-overlay {
    position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
    box-shadow: inset 0 0 100px rgba(255,0,0,0.3);
    pointer-events: none; opacity: 0; z-index: 999;
    transition: opacity 0.3s;
  }
  .warning-overlay.active { animation: flashDanger 1.5s infinite alternate; }
  @keyframes flashDanger { 0% { opacity: 0.1; } 100% { opacity: 0.4; } }

  /* ===== HEADER ===== */
  .header {
    background: #fff;
    padding: 14px 28px;
    border-bottom: 1px solid rgba(0,0,0,0.08);
    display: flex;
    align-items: center;
    justify-content: space-between;
    position: sticky;
    top: 0;
    z-index: 100;
    box-shadow: 0 2px 12px rgba(0,0,0,0.06);
  }
  .header h1 {
    font-size: 1.4em;
    font-weight: 800;
    background: linear-gradient(135deg, var(--accent), #ff6b81, #42a5f5);
    background-size: 200% 200%;
    animation: gradShift 4s ease infinite;
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    letter-spacing: -0.5px;
  }
  @keyframes gradShift { 0%,100%{background-position:0% 50%} 50%{background-position:100% 50%} }
  .conn-badge {
    display: flex;
    align-items: center;
    gap: 10px;
    font-size: 0.8em;
    color: var(--muted);
  }
  .conn-dot {
    width: 8px; height: 8px;
    border-radius: 50%;
    background: #4caf50;
    animation: pulse 2s infinite;
  }
  .header-meta {
    font-size: 0.72em;
    color: var(--muted);
    opacity: 0.7;
    font-weight: 500;
    font-variant-numeric: tabular-nums;
  }
  .latency-badge {
    font-size: 0.65em;
    padding: 2px 6px;
    border-radius: 4px;
    background: rgba(255,255,255,0.05);
    color: var(--muted);
    font-weight: 600;
    font-variant-numeric: tabular-nums;
  }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.3} }

  /* ===== LAYOUT ===== */
  .layout {
    display: grid;
    grid-template-columns: 260px 1fr 260px;
    gap: 14px;
    padding: 14px;
    max-width: 1600px;
    margin: 0 auto;
    transition: transform 0.4s cubic-bezier(0.4,0,0.2,1);
  }
  .layout.drawer-open {
    transform: translateX(100px);
  }
  .col { display: flex; flex-direction: column; gap: 12px; }

  /* ===== CARDS ===== */
  .card {
    background: var(--surface);
    border-radius: var(--radius);
    padding: 20px;
    box-shadow: 0 8px 24px rgba(0,0,0,0.05);
    border: 1px solid rgba(0,0,0,0.05);
  }
  .card::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
    background: linear-gradient(90deg, transparent, var(--accent), transparent);
    opacity: 0;
    transition: opacity 0.3s;
  }
  .card:hover {
    border-color: rgba(233,69,96,0.3);
    box-shadow: 0 15px 35px rgba(233,69,96,0.15), inset 0 0 20px rgba(255,255,255,0.02);
    transform: translateY(-4px) scale(1.02);
  }
  .card:hover::before { opacity: 1; }
  .card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: var(--muted);
    margin-bottom: 12px;
    font-weight: 700;
  }

  /* ===== STATE BADGE ===== */
  .state-badge {
    display: inline-block;
    padding: 6px 14px;
    border-radius: 8px;
    font-size: 1em;
    font-weight: 700;
    letter-spacing: 0.5px;
    transition: all 0.3s;
  }
  .state-LANE_FOLLOW { background: #1b5e20; color: #a5d6a7; }
  .state-OBSTRUCTION { background: #e65100; color: #ffcc80; }
  .state-ROUNDABOUT { background: #4a148c; color: #ce93d8; }
  .state-BOOM_GATE_1,.state-BOOM_GATE_2 { background: #b71c1c; color: #ef9a9a; }
  .state-TUNNEL { background: #263238; color: #90a4ae; }
  .state-HILL { background: #33691e; color: #aed581; }
  .state-BUMPER { background: #795548; color: #d7ccc8; }
  .state-TRAFFIC_LIGHT { background: #f57f17; color: #fff9c4; }
  .state-PARALLEL_PARK,.state-PERPENDICULAR_PARK { background: #0d47a1; color: #90caf9; }
  .state-DRIVE_TO_PERP { background: #1565c0; color: #90caf9; }
  .state-FINISHED { background: #ffd600; color: #333; }

  .mode-badge {
    padding: 4px 12px;
    border-radius: 6px;
    font-weight: 700;
    font-size: 0.75em;
    letter-spacing: 1px;
    transition: all 0.3s;
    vertical-align: middle;
  }
  .mode-AUTO { background: #4caf50; color: #fff; box-shadow: 0 0 8px rgba(76,175,80,0.3); }
  .mode-MANUAL { background: #d32f2f; color: #fff; box-shadow: 0 0 8px rgba(211,47,47,0.3); }

  .lap-badge {
    padding: 4px 10px;
    border-radius: 6px;
    font-weight: 600;
    font-size: 0.8em;
    background: var(--accent2);
    color: var(--accent);
  }
  .info-row {
    display: flex;
    gap: 16px;
    margin-top: 8px;
    font-size: 0.75em;
    color: var(--muted);
  }

  /* ===== TRAFFIC LIGHT ===== */
  .traffic-light {
    display: flex; gap: 12px; align-items: center; justify-content: center;
    background: linear-gradient(145deg, #555, #444);
    padding: 14px 24px; border-radius: 40px;
    width: fit-content; margin: 0 auto;
    box-shadow: inset 0 2px 6px rgba(0,0,0,0.3), 0 2px 8px rgba(0,0,0,0.1);
    border: 1px solid rgba(0,0,0,0.1);
  }
  .tl-circle {
    width: 32px; height: 32px; border-radius: 50%;
    border: 2px solid #000; transition: all 0.4s cubic-bezier(0.175, 0.885, 0.32, 1.275);
    box-shadow: inset 0 2px 6px rgba(0,0,0,0.8);
  }
  .tl-red { background: #3a1111; }
  .tl-yellow { background: #3a3311; }
  .tl-green { background: #113a11; }

  .tl-red.active { 
    background: radial-gradient(circle at 30% 30%, #ff8a80, #f44336); 
    box-shadow: 0 0 20px #f44336, 0 0 40px rgba(244,67,54,0.4), inset 0 -2px 6px rgba(0,0,0,0.3);
    border-color: #ff5252;
  }
  .tl-yellow.active { 
    background: radial-gradient(circle at 30% 30%, #fff59d, #ffeb3b); 
    box-shadow: 0 0 20px #ffeb3b, 0 0 40px rgba(255,235,59,0.4), inset 0 -2px 6px rgba(0,0,0,0.3);
    border-color: #ffff00;
  }
  .tl-green.active { 
    background: radial-gradient(circle at 30% 30%, #a5d6a7, #4caf50); 
    box-shadow: 0 0 20px #4caf50, 0 0 40px rgba(76,175,80,0.4), inset 0 -2px 6px rgba(0,0,0,0.3);
    border-color: #69f0ae;
  }
  .tl-label { text-align: center; margin-top: 8px; font-size: 0.75em; color: var(--muted); font-weight: 600; text-transform: uppercase; letter-spacing: 1px; }

  /* ===== SENSOR ROWS ===== */
  .s-row {
    display: flex;
    align-items: center;
    padding: 5px 0;
    border-bottom: 1px solid rgba(255,255,255,0.03);
  }
  .s-row:last-child { border: none; }
  .s-label { flex: 1; font-size: 0.78em; color: #999; }
  .s-val { font-weight: 600; font-size: 0.82em; }
  .dot {
    width: 10px; height: 10px;
    border-radius: 50%;
    display: inline-block;
    margin-right: 5px;
    transition: all 0.3s;
  }
  .dot-green { background: #4caf50; box-shadow: 0 0 6px #4caf50; }
  .dot-red { background: #f44336; box-shadow: 0 0 6px #f44336; }
  .dot-gray { background: #333; }

  /* ===== METER BARS ===== */
  .meter { height: 8px; background: rgba(26,26,46,0.8); border-radius: 4px; overflow: hidden; margin-top: 6px; box-shadow: inset 0 1px 3px rgba(0,0,0,0.3); }
  .meter-fill { height: 100%; border-radius: 4px; transition: width 0.4s cubic-bezier(0.4,0,0.2,1), background 0.4s; position: relative; }
  .meter-fill::after { content: ''; position: absolute; top: 0; left: 0; right: 0; height: 50%; background: linear-gradient(180deg, rgba(255,255,255,0.15), transparent); border-radius: 4px 4px 0 0; }
  .meter-blue { background: linear-gradient(90deg, #1565c0, #42a5f5); }
  .meter-orange { background: linear-gradient(90deg, #e65100, #ff9800, #ffb74d); }

  /* ===== SPEED GAUGE ===== */
  .speed-display {
    text-align: center;
    padding: 12px 0;
  }
  .speed-display .num {
    font-size: 2.8em;
    font-weight: 800;
    background: linear-gradient(135deg, #42a5f5, #1e88e5, #e94560);
    background-size: 200% 200%;
    animation: gradShift 3s ease infinite;
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    line-height: 1;
    transition: all 0.3s;
  }
  .speed-display .unit {
    font-size: 0.6em;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 3px;
    margin-top: 4px;
  }
  .gear-dots {
    display: flex; justify-content: center; gap: 6px; margin-top: 8px;
  }
  .gear-dot {
    width: 12px; height: 12px; border-radius: 50%; background: rgba(255,255,255,0.08);
    border: 1px solid rgba(255,255,255,0.1);
    transition: all 0.4s cubic-bezier(0.175, 0.885, 0.32, 1.275);
  }
  .gear-dot.active {
    background: var(--accent); border-color: #fff;
    box-shadow: 0 0 15px rgba(233,69,96,0.8), 0 0 30px rgba(233,69,96,0.4);
    transform: scale(1.3);
  }

  /* ===== CHALLENGE SELECTOR ===== */
  .ctrl-state-badge {
    display: inline-block;
    padding: 4px 10px;
    border-radius: 6px;
    font-weight: 700;
    font-size: 0.78em;
    background: #4a148c;
    color: #fff;
    border: 1px solid rgba(206,147,216,0.2);
  }

  /* ===== CAMERA ===== */
  .cam-card {
    display: flex;
    flex-direction: column;
  }
  .cam-toggle {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 8px 16px;
    border-radius: 8px;
    border: 1px solid rgba(255,255,255,0.1);
    background: rgba(255,255,255,0.05);
    color: #42a5f5;
    cursor: pointer;
    font-size: 0.8em;
    font-weight: 600;
    transition: all 0.2s;
    margin-bottom: 10px;
    font-family: inherit;
  }
  .cam-toggle:hover { background: rgba(66,165,245,0.15); border-color: #42a5f5; }
  .cam-toggle.on { background: rgba(76,175,80,0.2); color: #4caf50; border-color: #4caf50; }
  .cam-container {
    width: 100%;
    display: flex;
    align-items: center;
    justify-content: center;
    background: #050508;
    border-radius: 12px;
    overflow: hidden;
    position: relative;
  }
  .cam-container img {
    width: 100%;
    height: 100%;
    object-fit: contain;
    border-radius: 8px;
  }
  .cam-container.active { border-color: rgba(66,165,245,0.4); box-shadow: 0 0 20px rgba(66,165,245,0.1) inset; }
  .cam-off { display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100%; color: #555; font-size: 0.85em; }
  .cam-off .icon { font-size: 2em; margin-bottom: 8px; opacity: 0.5; }
  .cam-tabs { display: flex; gap: 4px; background: rgba(0,0,0,0.05); padding: 4px; border-radius: 8px; margin-bottom: 8px; }
  .cam-tab { flex: 1; text-align: center; font-size: 0.7em; padding: 6px 0; border-radius: 6px; cursor: pointer; color: #5c5f77; transition: all 0.2s; font-weight: 600; }
  .cam-tab:hover { background: rgba(0,0,0,0.05); color: #4c4f69; }
  .cam-tab.active { background: var(--accent); color: #fff; }
  
  /* ===== FLOW TIMELINE ===== */
  .ctrl-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 4px;
    text-align: center;
  }
  .ctrl-btn {
    padding: 6px 2px;
    border-radius: 6px;
    background: rgba(0,0,0,0.04);
    border: 1px solid rgba(0,0,0,0.06);
    font-size: 0.7em;
    color: #6c6f85;
    transition: all 0.15s;
    font-weight: 600;
  }
  .ctrl-btn.active {
    background: var(--accent);
    color: #fff;
    border-color: var(--accent);
    box-shadow: 0 0 8px rgba(30,102,245,0.3);
  }
  .joy-info {
    margin-top: 14px;
    display: flex;
    justify-content: center;
    gap: 30px;
  }
  .joy-container {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 6px;
  }
  .joy-circle {
    width: 60px; height: 60px;
    border-radius: 50%;
    background: #dce0e8;
    border: 1px solid rgba(0,0,0,0.1);
    position: relative;
  }
  .joy-dot {
    width: 14px; height: 14px;
    background: var(--muted);
    border-radius: 50%;
    position: absolute;
    top: 50%; left: 50%;
    transform: translate(-50%, -50%);
    box-shadow: 0 0 8px rgba(0,0,0,0.1);
    transition: background 0.2s;
  }
  .joy-dot.active {
    background: var(--accent);
    box-shadow: 0 0 12px var(--accent);
  }
  .joy-label {
    font-size: 0.65em;
    color: var(--muted);
    font-weight: 600;
  }
  .btn-debug {
    margin-top: 6px;
    padding: 4px 6px;
    background: var(--surface2);
    border-radius: 4px;
    font-size: 0.65em;
    font-family: 'Courier New', monospace;
    font-weight: 700;
    color: var(--text);
    min-height: 18px;
  }

  /* ===== ODOM ===== */
  .odom-big {
    font-size: 1.6em;
    font-weight: 700;
    color: #42a5f5;
  }
  .odom-unit { font-size: 0.5em; color: var(--muted); margin-left: 3px; }

  /* ===== RESPONSIVE ===== */
  @media (max-width: 960px) {
    .layout { grid-template-columns: 1fr; }
    .cam-container { min-height: 200px; }
    .flow-bar { overflow-x: auto; }
  }

  /* ===== COMPETITION FLOW ===== */
  .flow-section {
    max-width: 1400px;
    margin: 0 auto;
    padding: 0 14px 14px;
  }
  .flow-card {
    background: var(--card);
    backdrop-filter: blur(16px);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 18px;
    position: relative;
    overflow: hidden;
  }
  .flow-card::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
    background: linear-gradient(90deg, transparent, #42a5f5, transparent);
    opacity: 0.5;
  }
  .flow-card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: var(--muted);
    margin-bottom: 14px;
    font-weight: 700;
  }
  .flow-bar {
    display: flex;
    align-items: center;
    flex-wrap: wrap;
    gap: 6px;
    padding: 8px 0;
  }
  .flow-node {
    padding: 6px 10px;
    border-radius: 8px;
    font-size: 0.68em;
    font-weight: 600;
    letter-spacing: 0.2px;
    background: rgba(255,255,255,0.04);
    border: 1px solid rgba(255,255,255,0.06);
    color: #555;
    transition: all 0.4s cubic-bezier(0.4,0,0.2,1);
    cursor: default;
    white-space: nowrap;
    position: relative;
  }
  .flow-node.done {
    background: rgba(76,175,80,0.12);
    border-color: rgba(76,175,80,0.3);
    color: #81c784;
  }
  .flow-node.done::after {
    content: '✓';
    position: absolute;
    top: -4px; right: -4px;
    width: 14px; height: 14px;
    background: #4caf50;
    border-radius: 50%;
    font-size: 9px;
    display: flex; align-items: center; justify-content: center;
    color: #fff;
  }
  .flow-node.active {
    background: linear-gradient(135deg, rgba(233,69,96,0.2), rgba(66,165,245,0.2));
    border-color: var(--accent);
    color: #fff;
    box-shadow: 0 0 20px rgba(233,69,96,0.25), 0 0 40px rgba(233,69,96,0.1);
    transform: scale(1.08);
    font-weight: 700;
  }
  @keyframes activeGlow {
    0%,100% { box-shadow: 0 0 15px rgba(233,69,96,0.2); }
    50% { box-shadow: 0 0 25px rgba(233,69,96,0.4), 0 0 50px rgba(233,69,96,0.15); }
  }
  .flow-node.active { animation: activeGlow 2s ease infinite; }
  .flow-arrow {
    color: #333;
    font-size: 0.6em;
    padding: 0 1px;
    transition: color 0.3s;
  }
  .flow-arrow.passed { color: #4caf50; }
  .flow-lap-label {
    flex-shrink: 0;
    padding: 4px 10px;
    border-radius: 12px;
    font-size: 0.55em;
    font-weight: 700;
    letter-spacing: 1px;
    text-transform: uppercase;
    margin: 0 6px;
  }
  .lap1-label { background: rgba(233,69,96,0.15); color: var(--accent); border: 1px solid rgba(233,69,96,0.2); }
  .lap2-label { background: rgba(66,165,245,0.15); color: #42a5f5; border: 1px solid rgba(66,165,245,0.2); }

  /* ===== CONTROLLER POPOUT ===== */
  .ctrl-popout-tab {
    position: fixed;
    right: 0;
    top: 50%;
    transform: translateY(-50%);
    background: #fff;
    padding: 10px 8px;
    border-radius: 10px 0 0 10px;
    border: 1px solid rgba(0,0,0,0.08);
    border-right: none;
    cursor: pointer;
    z-index: 200;
    writing-mode: vertical-rl;
    text-orientation: mixed;
    font-size: 0.7em;
    font-weight: 700;
    color: var(--accent);
    letter-spacing: 1px;
    transition: all 0.3s;
    box-shadow: -2px 0 8px rgba(0,0,0,0.06);
  }
  .ctrl-popout-tab:hover {
    background: rgba(30,102,245,0.05);
    padding-right: 12px;
  }
  .ctrl-drawer {
    position: fixed;
    right: -320px;
    top: 60px;
    bottom: 0;
    width: 300px;
    background: #fff;
    border-left: 1px solid rgba(0,0,0,0.08);
    z-index: 199;
    transition: right 0.4s cubic-bezier(0.4,0,0.2,1);
    overflow-y: auto;
    padding: 20px;
    box-shadow: -4px 0 20px rgba(0,0,0,0.08);
  }
  .ctrl-drawer.open { right: 0; }
  .ctrl-drawer h3 {
    font-size: 0.7em;
    text-transform: uppercase;
    letter-spacing: 2px;
    color: var(--accent);
    margin-bottom: 14px;
    font-weight: 700;
  }
  .ctrl-map-row {
    display: flex;
    align-items: center;
    padding: 8px 0;
    border-bottom: 1px solid rgba(255,255,255,0.04);
  }
  .ctrl-map-row:last-child { border: none; }
  .ctrl-key {
    flex-shrink: 0;
    width: 60px;
    padding: 4px 8px;
    font-size: 0.75em;
    font-weight: 700;
    text-align: center;
    border-radius: 6px;
    background: var(--surface2);
    border: 1px solid rgba(0,0,0,0.1);
    color: #aaa;
    margin-right: 12px;
  }
  .ctrl-desc {
    font-size: 0.78em;
    color: #888;
  }
  .ctrl-section-label {
    font-size: 0.6em;
    text-transform: uppercase;
    letter-spacing: 2px;
    color: #444;
    margin: 16px 0 8px;
    padding-bottom: 4px;
    border-bottom: 1px solid rgba(255,255,255,0.05);
  }

  /* ===== EVENT LOG ===== */
  .log-section {
    max-width: 1400px;
    margin: 0 auto;
    padding: 0 14px 14px;
  }
  .log-card {
    background: var(--card);
    backdrop-filter: blur(16px);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 14px 18px;
  }
  .log-card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: var(--muted);
    margin-bottom: 10px;
    font-weight: 700;
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .event-log {
    font-family: 'JetBrains Mono', 'Consolas', monospace;
    font-size: 0.75em;
    height: 140px;
    overflow-y: auto;
    background: #e6e9ef;
    border-radius: 8px;
    padding: 12px;
    border: 1px solid rgba(0,0,0,0.05);
    color: #4c4f69;
  }
  .event-log::-webkit-scrollbar { width: 6px; }
  .event-log::-webkit-scrollbar-track { background: transparent; }
  .event-log::-webkit-scrollbar-thumb { background: rgba(0,0,0,0.1); border-radius: 3px; }
  .log-entry { margin-bottom: 4px; border-bottom: 1px solid rgba(0,0,0,0.02); padding-bottom: 4px; }
  .log-time { color: var(--accent); margin-right: 8px; font-weight: 600; }
  .log-val { font-weight: 700; color: #d20f39; }

  /* ===== PARAMETER DRAWER ===== */
  .param-popout-tab {
    position: fixed; left: 0; top: 50%; transform: translateY(-50%);
    background: #fff;
    padding: 10px 8px; border-radius: 0 10px 10px 0;
    border: 1px solid rgba(0,0,0,0.08); border-left: none;
    cursor: pointer; z-index: 200; writing-mode: vertical-lr;
    text-orientation: mixed; font-size: 0.7em; font-weight: 700;
    color: var(--accent); letter-spacing: 1px; transition: all 0.3s;
    box-shadow: 2px 0 8px rgba(0,0,0,0.06);
  }
  .param-popout-tab:hover { background: rgba(30,102,245,0.05); padding-left: 12px; }
  .param-drawer {
    position: fixed; left: -480px; top: 60px; bottom: 0; width: 420px;
    background: #fff;
    border-right: 1px solid rgba(0,0,0,0.08); z-index: 199;
    transition: left 0.4s cubic-bezier(0.4,0,0.2,1); overflow-y: auto;
    padding: 20px; box-shadow: 4px 0 20px rgba(0,0,0,0.08);
  }
  .param-drawer.open { left: 0; }
  .param-drawer h3 {
    font-size: 0.85em; text-transform: uppercase; letter-spacing: 2px;
    color: var(--accent); margin-bottom: 8px; font-weight: 700;
  }
  .param-drawer .note { font-size: 0.8em; color: #666; margin-bottom: 12px; }
  .param-card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: #666;
    margin-bottom: 12px;
    font-weight: 700;
    display: flex;
    align-items: center;
    justify-content: space-between;
  }
  .param-toolbar {
    display: flex; gap: 8px; align-items: center;
    margin-bottom: 12px;
  }
  .param-refresh-btn, .param-load-btn {
    padding: 5px 14px;
    border-radius: 6px;
    border: 1px solid rgba(30,102,245,0.2);
    background: rgba(30,102,245,0.05);
    color: var(--accent);
    cursor: pointer;
    font-size: 0.75em;
    font-weight: 700;
    transition: all 0.2s;
    font-family: inherit;
  }
  .param-refresh-btn:hover, .param-load-btn:hover {
    background: rgba(30,102,245,0.1); border-color: var(--accent);
  }
  .param-name[title] {
    position: relative;
    cursor: help;
    border-bottom: 1px dotted rgba(0,0,0,0.25);
  }
  .param-name[title]:hover::after {
    content: attr(title);
    position: fixed;
    transform: translate(0, 24px);
    background: #333;
    border: 1px solid rgba(0,0,0,0.3);
    color: #fff;
    padding: 6px 10px;
    border-radius: 6px;
    font-size: 0.85em;
    font-weight: 500;
    max-width: 300px;
    white-space: normal;
    z-index: 99999;
    pointer-events: none;
    box-shadow: 0 4px 12px rgba(0,0,0,0.15);
    animation: tipFade 0.15s ease;
  }
  @keyframes tipFade { from { opacity: 0; transform: translateY(-2px); } to { opacity: 1; transform: translateY(0); } }
  .param-node-block {
    margin-bottom: 8px;
    border: 1px solid rgba(0,0,0,0.08);
    border-radius: 10px;
    overflow: hidden;
  }
  .param-node-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 14px;
    background: #f8f9fb;
    cursor: pointer;
    transition: background 0.2s;
  }
  .param-node-header:hover { background: #f0f1f5; }
  .param-node-name {
    font-size: 0.8em;
    font-weight: 700;
    color: var(--accent);
  }
  .param-node-count {
    font-size: 0.65em;
    color: #888;
    font-weight: 600;
  }
  .param-node-body {
    max-height: 0;
    overflow: hidden;
    transition: max-height 0.35s ease;
  }
  .param-node-body.open {
    max-height: 2000px;
  }
  .param-row {
    display: grid;
    grid-template-columns: 1fr 80px 38px 38px;
    align-items: center;
    padding: 6px 14px;
    border-top: 1px solid rgba(0,0,0,0.04);
    gap: 6px;
    position: relative;
  }
  .param-name {
    font-size: 0.78em;
    color: #333;
    font-weight: 600;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .param-val {
    width: 100%;
    box-sizing: border-box;
    padding: 5px 8px;
    border-radius: 4px;
    border: 1px solid rgba(0,0,0,0.12);
    background: #f8f9fb;
    color: #333;
    font-size: 0.8em;
    font-family: 'Segoe UI', Roboto, sans-serif;
    outline: none;
    transition: border-color 0.2s;
  }
  .param-val:focus { border-color: var(--accent); box-shadow: 0 0 0 2px rgba(30,102,245,0.15); }
  .param-set-btn {
    padding: 4px 0;
    border-radius: 4px;
    border: 1px solid rgba(64,160,43,0.3);
    background: rgba(64,160,43,0.08);
    color: var(--success);
    cursor: pointer;
    font-size: 0.65em;
    font-weight: 700;
    transition: all 0.2s;
    font-family: inherit;
    text-align: center;
  }
  .param-set-btn:hover { background: rgba(64,160,43,0.15); border-color: var(--success); }
  .param-get-btn {
    padding: 4px 0;
    border-radius: 4px;
    border: 1px solid rgba(30,102,245,0.3);
    background: rgba(30,102,245,0.08);
    color: var(--accent);
    cursor: pointer;
    font-size: 0.65em;
    font-weight: 700;
    transition: all 0.2s;
    font-family: inherit;
    text-align: center;
  }
  .param-get-btn:hover { background: rgba(30,102,245,0.15); border-color: var(--accent); }
  .param-status {
    position: absolute;
    right: 14px;
    bottom: -8px;
    font-size: 0.65em;
    padding: 2px 6px;
    border-radius: 3px;
    animation: logFade 0.3s ease;
  }
  .param-status.ok { color: var(--success); }
  .param-status.err { color: var(--danger); }
  .param-empty {
    text-align: center;
    padding: 20px;
    color: #888;
    font-size: 0.8em;
  }
</style>
</head>
<body>

<!-- BACKGROUND ORBS -->
<div class="bg-orbs">
  <div class="orb orb-1"></div>
  <div class="orb orb-2"></div>
  <div class="orb orb-3"></div>
</div>

<!-- WARNING OVERLAY -->
<div id="warning-overlay" class="warning-overlay"></div>

<!-- HEADER -->
<div class="header">
  <h1>🤖 RISA-Bot <span style="font-size:0.6em;color:rgba(255,255,255,0.4);vertical-align:middle;">v2.0</span></h1>
  <div class="conn-badge">
    <span class="header-meta" id="uptimeText">00:00:00</span>
    <span class="latency-badge" id="latencyText">— ms</span>
    <span class="conn-dot" id="connDot"></span>
    <span id="connText">Connecting...</span>
  </div>
</div>

<!-- MAIN LAYOUT -->
<div class="layout">

  <!-- ===== LEFT COLUMN ===== -->
  <div class="col">

    <!-- State Machine -->
    <div class="card">
      <h3 style="display:flex;align-items:center;justify-content:space-between;">State Machine <span class="mode-badge" id="modeBadge">MANUAL</span></h3>
      <div style="margin-top:4px;">
        <span class="state-badge" id="stateBadge">—</span>
      </div>
      <div style="margin-top:8px;">
        <span class="lap-badge" id="lapBadge">Lap —</span>
      </div>
      <div class="info-row">
        <span>⏱ <span id="stateTime">0</span>s</span>
        <span>📏 <span id="stateDist">0.00</span>m</span>
        <span>🏁 <span id="lapTimer" style="font-variant-numeric:tabular-nums;">00:00</span></span>
      </div>
      <div id="stopReasonRow" style="margin-top:6px;">
        <span id="stopBadge" style="display:inline-block;padding:3px 10px;border-radius:4px;font-size:0.75em;font-weight:700;letter-spacing:0.5px;background:rgba(166,227,161,0.15);color:#a6e3a1;">DRIVING</span>
      </div>
    </div>

    <!-- Traffic Light -->
    <div class="card">
      <h3>Traffic Light</h3>
      <div class="traffic-light">
        <div class="tl-circle tl-red" id="tlRed"></div>
        <div class="tl-circle tl-yellow" id="tlYellow"></div>
        <div class="tl-circle tl-green" id="tlGreen"></div>
      </div>
      <div class="tl-label" id="tlText">unknown</div>
    </div>

    <!-- Manual Control -->
    <div class="card">
      <h3>Manual Control</h3>
      <div class="speed-display">
        <div class="num" id="speedPct">25%</div>
        <div class="unit">Drive Speed</div>
        <div class="gear-dots">
          <div class="gear-dot active" id="gear0"></div>
          <div class="gear-dot" id="gear1"></div>
          <div class="gear-dot" id="gear2"></div>
          <div class="gear-dot" id="gear3"></div>
        </div>
      </div>
      <div class="meter"><div class="meter-fill meter-orange" id="speedBar" style="width:25%"></div></div>
      <div style="margin-top:6px;font-size:0.65em;color:#444;text-align:center;">D-pad ▲/▼ to shift</div>
      <div style="margin-top:12px;">
        <div class="s-row">
          <span class="s-label">Selector</span>
          <span class="ctrl-state-badge" id="ctrlState">LANE_FOLLOW</span>
        </div>
        <div style="font-size:0.65em;color:#444;">LB / RB to cycle</div>
      </div>
    </div>

    <!-- Lane Following -->
    <div class="card">
      <h3>Lane Following</h3>
      <div class="s-row">
        <span class="s-label">Error</span>
        <span class="s-val" id="laneErr">0.000</span>
      </div>
      <div class="meter"><div class="meter-fill meter-blue" id="laneBar" style="width:50%"></div></div>
      <div class="s-row" style="margin-top:6px;">
        <span class="s-label">Linear X</span>
        <span class="s-val" id="cmdLinX">0.000</span>
      </div>
      <div class="s-row">
        <span class="s-label">Angular Z</span>
        <span class="s-val" id="cmdAngZ">0.000</span>
      </div>
    </div>

  </div>

  <!-- ===== CENTER COLUMN — CAMERA ===== -->
  <div class="col">
    <div class="card cam-card">
      <h3>Camera Feed</h3>
      <button class="cam-toggle" id="camBtn" onclick="toggleCam()">📷 Enable Camera</button>
      <div class="cam-tabs" id="camTabs" style="display:none;">
        <div class="cam-tab active" onclick="setCamView('raw', this)">Raw</div>
        <div class="cam-tab" onclick="setCamView('line_follower', this)">Lane Lines</div>
        <div class="cam-tab" onclick="setCamView('traffic_light', this)">Traffic Light</div>
        <div class="cam-tab" onclick="setCamView('obstacle', this)">Obstacle</div>
      </div>
      <div class="cam-container" id="camContainer">
        <div class="cam-off" id="camOff">
          <div class="icon">📷</div>
          Click above to enable
        </div>
        <img id="camImg" src="" alt="Camera" style="display:none;">
      </div>
    </div>
  </div>

  <!-- ===== RIGHT COLUMN ===== -->
  <div class="col">

    <!-- Sensors -->
    <div class="card">
      <h3>Sensors</h3>
      <div class="s-row"><span class="s-label">LiDAR</span><span class="s-val"><span class="dot dot-gray" id="dotLidar"></span><span id="valLidar">—</span></span></div>
      <div class="s-row"><span class="s-label">Camera</span><span class="s-val"><span class="dot dot-gray" id="dotCam"></span><span id="valCam">—</span></span></div>
      <div class="s-row"><span class="s-label">Fused</span><span class="s-val"><span class="dot dot-gray" id="dotFused"></span><span id="valFused">—</span></span></div>
      <div class="s-row"><span class="s-label">Boom Gate</span><span class="s-val"><span class="dot dot-gray" id="dotGate"></span><span id="valGate">—</span></span></div>
      <div class="s-row"><span class="s-label">Tunnel</span><span class="s-val"><span class="dot dot-gray" id="dotTunnel"></span><span id="valTunnel">—</span></span></div>
      <div class="s-row"><span class="s-label">Obstruction</span><span class="s-val"><span class="dot dot-gray" id="dotObst"></span><span id="valObst">—</span></span></div>
      <div class="s-row"><span class="s-label">Parking</span><span class="s-val"><span class="dot dot-gray" id="dotPark"></span><span id="valPark">—</span></span></div>
    </div>

    <!-- Odometry -->
    <div class="card">
      <h3>Odometry</h3>
      <div class="odom-big"><span id="odomDist">0.00</span><span class="odom-unit">m</span></div>
      <div class="s-row" style="margin-top:6px;">
        <span class="s-label">Speed</span>
        <span class="s-val" id="odomSpeed">0.000 m/s</span>
      </div>
    </div>

    <!-- Controller -->
    <div class="card">
      <h3>Controller</h3>
      <div class="ctrl-grid">
        <div class="ctrl-btn" id="btnLB">LB</div>
        <div class="ctrl-btn" id="btnUp">▲</div>
        <div class="ctrl-btn" id="btnRB">RB</div>
        <div class="ctrl-btn" id="btnLeft">◀</div>
        <div class="ctrl-btn" id="btnStart">STA</div>
        <div class="ctrl-btn" id="btnRight">▶</div>
        <div class="ctrl-btn" id="btnX">X</div>
        <div class="ctrl-btn" id="btnDown">▼</div>
        <div class="ctrl-btn" id="btnY">Y</div>
        <div class="ctrl-btn" id="btnA">A</div>
        <div class="ctrl-btn" id="btnLT">LT</div>
        <div class="ctrl-btn" id="btnB">B</div>
        <div class="ctrl-btn" id="btnRT" style="grid-column:3;">RT</div>
      </div>
      <div class="joy-info">
        <div class="joy-container">
          <div class="joy-circle"><div class="joy-dot" id="joyDotL"></div></div>
          <div class="joy-label" id="joyValL">L: 0.0, 0.0</div>
        </div>
        <div class="joy-container">
          <div class="joy-circle"><div class="joy-dot" id="joyDotR"></div></div>
          <div class="joy-label" id="joyValR">R: 0.0, 0.0</div>
        </div>
      </div>
      <div class="btn-debug" id="btnDebug">Press a button to see index…</div>
    </div>

  </div>
</div>

<!-- ===== BEHAVIOR PRIORITY VISUALIZER ===== -->
<div class="flow-section">
  <div class="flow-card">
    <h3>🧠 Hybrid Action Priority <span style="font-size:0.85em;color:#444;font-weight:400;text-transform:none;letter-spacing:0;"> — Highlighted block is the currently active overriding behavior</span></h3>
    <div class="flow-bar" id="flowBar">
      <div class="flow-node" id="flow_LANE_FOLLOW">Lane Follow</div>
      <span class="flow-arrow">◀</span>
      <div class="flow-node" id="flow_TUNNEL">Tunnel</div>
      <span class="flow-arrow">◀</span>
      <div class="flow-node" id="flow_OBSTRUCTION">Obstruction</div>
      <span class="flow-arrow">◀</span>
      <div class="flow-node" id="flow_BOOM_GATE">Boom Gate</div>
      <span class="flow-arrow">◀</span>
      <div class="flow-node" id="flow_TRAFFIC_LIGHT">Traffic Light</div>
      <span class="flow-arrow">◀</span>
      <div class="flow-node" id="flow_MANUAL">Manual Control</div>
    </div>
  </div>
</div>

<!-- ===== EVENT LOG ===== -->
<div class="log-section">
  <div class="log-card">
    <h3><span style="opacity:0.6">📝</span> Event Log</h3>
    <div class="log-scroll" id="logScroll">
      <div class="log-entry"><span class="log-time">--:--:--</span><span class="log-event">Dashboard started</span></div>
    </div>
  </div>
</div>

<!-- ===== PARAMETER TUNING ===== -->
<!-- ===== PARAMETER TUNING DRAWER ===== -->
<div class="param-popout-tab" onclick="toggleParamDrawer()">⚙️ Parameters</div>
<div class="param-drawer" id="paramDrawer">
  <h3>⚙️ Parameter Tuning</h3>
  <div class="note">💡 Changes apply instantly to nodes but revert to defaults upon restart.</div>
  <div id="paramContainer"></div>
</div>

<!-- ===== CONTROLLER POPOUT ===== -->
<div class="ctrl-popout-tab" onclick="toggleCtrlDrawer()">🎮 Controls</div>
<div class="ctrl-drawer" id="ctrlDrawer">
  <h3>🎮 Control Mapping</h3>
  <div class="ctrl-section-label">Driving</div>
  <div class="ctrl-map-row"><span class="ctrl-key">L Stick Y</span><span class="ctrl-desc">Throttle (forward/reverse)</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">R Stick X</span><span class="ctrl-desc">Steering (left/right)</span></div>
  <div class="ctrl-section-label">Speed</div>
  <div class="ctrl-map-row"><span class="ctrl-key">D-Pad ▲</span><span class="ctrl-desc">Speed up (shift gear)</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">D-Pad ▼</span><span class="ctrl-desc">Speed down (shift gear)</span></div>
  <div class="ctrl-section-label">Mode</div>
  <div class="ctrl-map-row"><span class="ctrl-key">Y</span><span class="ctrl-desc">Toggle Auto/Manual mode</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">Start</span><span class="ctrl-desc">Toggle Auto/Manual mode</span></div>
  <div class="ctrl-section-label">Challenge</div>
  <div class="ctrl-map-row"><span class="ctrl-key">RB</span><span class="ctrl-desc">Next challenge state</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">LB</span><span class="ctrl-desc">Previous challenge state</span></div>
  <div class="ctrl-section-label">Camera</div>
  <div class="ctrl-map-row"><span class="ctrl-key">R Stick Y</span><span class="ctrl-desc">Camera tilt (if enabled)</span></div>
</div>

<script>
let eventLog = [];
const PRIORITY_ORDER = ['LANE_FOLLOW', 'TUNNEL', 'OBSTRUCTION', 'BOOM_GATE', 'TRAFFIC_LIGHT', 'MANUAL'];
let lastState = '';
let lastLap = 0;
let lapStartTime = Date.now();
const dashStartTime = Date.now();
let lastStopReason = '';

// Session uptime timer
setInterval(() => {
  const elapsed = Math.floor((Date.now() - dashStartTime) / 1000);
  const h = String(Math.floor(elapsed / 3600)).padStart(2, '0');
  const m = String(Math.floor((elapsed % 3600) / 60)).padStart(2, '0');
  const s = String(elapsed % 60).padStart(2, '0');
  document.getElementById('uptimeText').textContent = h + ':' + m + ':' + s;
  // Lap timer
  const lapElapsed = Math.floor((Date.now() - lapStartTime) / 1000);
  const lm = String(Math.floor(lapElapsed / 60)).padStart(2, '0');
  const ls = String(lapElapsed % 60).padStart(2, '0');
  document.getElementById('lapTimer').textContent = lm + ':' + ls;
}, 1000);

function toggleCtrlDrawer() {
  document.getElementById('ctrlDrawer').classList.toggle('open');
}

function toggleParamDrawer() {
  document.getElementById('paramDrawer').classList.toggle('open');
  document.querySelector('.layout').classList.toggle('drawer-open');
}

function addLogEntry(text) {
  const now = new Date();
  const ts = now.toLocaleTimeString('en-GB');
  eventLog.unshift({time: ts, text: text});
  if (eventLog.length > 30) eventLog.pop();
  const container = document.getElementById('logScroll');
  container.innerHTML = eventLog.map(e =>
    `<div class="log-entry"><span class="log-time">${e.time}</span><span class="log-event">${e.text}</span></div>`
  ).join('');
}

function updateFlow(currentState) {
  const activeIdx = PRIORITY_ORDER.indexOf(currentState);
  
  PRIORITY_ORDER.forEach((s, i) => {
    const el = document.getElementById('flow_' + s);
    if (!el) return;
    
    el.classList.remove('active', 'done');
    // If it's the active state, highlight it
    if (i === activeIdx) {
      el.classList.add('active');
    } 
    // If it's a lower priority state than current, mark it as "done" (overridden)
    else if (i < activeIdx) {
      el.classList.add('done');
    }
  });

  // Color arrows. Arrow i is between node i and i+1.
  const arrows = document.querySelectorAll('.flow-arrow');
  arrows.forEach((a, i) => {
    // If the active state is higher than i, the arrow is lit up implying flow of priority
    a.classList.toggle('passed', i < activeIdx);
  });
}

function toggleCam() {
  const b = document.getElementById('camBtn');
  const d = document.getElementById('camContainer');
  const t = document.getElementById('camTabs');
  const off = document.getElementById('camOff');
  const img = document.getElementById('camImg');
  if(b.classList.contains('active')) {
    b.classList.remove('active');
    b.textContent = '📷 Enable Camera';
    d.classList.remove('active');
    t.style.display = 'none';
    off.style.display = 'flex';
    img.style.display = 'none';
    img.src = '';
  } else {
    b.classList.add('active');
    b.textContent = '⏹ Disable Camera';
    d.classList.add('active');
    t.style.display = 'flex';
    off.style.display = 'none';
    img.style.display = 'block';
    img.src = '/camera_feed?' + new Date().getTime();
  }
}

function setCamView(view, btn) {
  document.querySelectorAll('.cam-tab').forEach(t => t.classList.remove('active'));
  btn.classList.add('active');
  fetch('/api/set_cam_view?view=' + encodeURIComponent(view));
}

let last_data_time = 0;
function update() {
  const fetchStart = performance.now();
  fetch('/data')
    .then(r => r.json())
    .then(d => {
      const latency = Math.round(performance.now() - fetchStart);
      document.getElementById('latencyText').textContent = latency + ' ms';
      document.getElementById('connDot').style.background = '#4caf50';
      document.getElementById('connText').textContent = 'Connected';

      // State
      const sb = document.getElementById('stateBadge');
      const currentState = d.state;
      sb.textContent = currentState;
      sb.className = 'state-badge state-' + currentState;

      // Update competition flow
      updateFlow(currentState);

      // Log state changes
      if (currentState !== lastState && lastState !== '') {
        addLogEntry(`State: <span class="log-val">${lastState}</span> → <span class="log-val">${currentState}</span>`);
      }
      lastState = currentState;

      const mb = document.getElementById('modeBadge');
      const newMode = d.auto_mode ? 'AUTO' : 'MANUAL';
      const oldMode = mb.textContent;
      mb.textContent = newMode;
      mb.className = 'mode-badge mode-' + newMode;
      if (oldMode && oldMode !== newMode) {
        addLogEntry(`Mode: <span class="log-val">${newMode}</span>`);
      }

      document.getElementById('lapBadge').textContent = 'Lap ' + d.lap;
      // Lap timer reset
      if (d.lap !== lastLap && lastLap !== 0) {
        lapStartTime = Date.now();
        addLogEntry(`Lap <span class="log-val">${d.lap}</span> started`);
      }
      lastLap = d.lap;
      document.getElementById('stateTime').textContent = d.state_time;
      document.getElementById('stateDist').textContent = d.state_dist;

      // Stop reason badge
      const stopEl = document.getElementById('stopBadge');
      const sr = d.stop_reason || '';
      if (sr.length > 0) {
        stopEl.textContent = '⛔ ' + sr;
        stopEl.style.background = 'rgba(243,139,168,0.2)';
        stopEl.style.color = '#f38ba8';
        if (sr !== lastStopReason && lastStopReason === '') {
          addLogEntry(`⛔ Stopped: <span class="log-val">${sr}</span>`);
        }
      } else {
        stopEl.textContent = 'DRIVING';
        stopEl.style.background = 'rgba(64,160,43,0.15)';
        stopEl.style.color = '#40a02b';
        if (lastStopReason && lastStopReason.length > 0) {
          addLogEntry(`✅ Resumed driving`);
        }
      }
      lastStopReason = sr;

      // Traffic light
      ['Red','Yellow','Green'].forEach(c => {
        document.getElementById('tl'+c).classList.toggle('active', d.traffic_light === c.toLowerCase());
      });
      document.getElementById('tlText').textContent = d.traffic_light;

      // Sensors
      function ss(dId, vId, v, tl, fl) {
        const dot = document.getElementById(dId), val = document.getElementById(vId);
        if (v===null||v===undefined){dot.className='dot dot-gray';val.textContent='—';return;}
        dot.className = v ? 'dot dot-red' : 'dot dot-green';
        val.textContent = v ? (tl||'YES') : (fl||'NO');
      }
      ss('dotLidar','valLidar',d.lidar_obstacle,'BLOCKED','CLEAR');
      ss('dotCam','valCam',d.camera_obstacle,'BLOCKED','CLEAR');
      ss('dotFused','valFused',d.fused_obstacle,'BLOCKED','CLEAR');
      ss('dotTunnel','valTunnel',d.tunnel_detected,'IN TUNNEL','NO');
      ss('dotObst','valObst',d.obstruction_active,'DODGING','NO');
      ss('dotPark','valPark',d.parking_complete,'DONE','NO');

      // Warning Flash Overlay (if blocked in AUTO)
      const isBlocked = d.fused_obstacle; // Use fused for reliable alarm
      const overlay = document.getElementById('warning-overlay');
      if (overlay) {
        if (d.auto_mode && isBlocked) overlay.classList.add('active');
        else overlay.classList.remove('active');
      }

      const gD=document.getElementById('dotGate'),gV=document.getElementById('valGate');
      if(d.boom_gate===null){gD.className='dot dot-gray';gV.textContent='—';}
      else{gD.className=d.boom_gate?'dot dot-green':'dot dot-red';gV.textContent=d.boom_gate?'OPEN':'CLOSED';}

      // Lane
      document.getElementById('laneErr').textContent = d.lane_error.toFixed(3);
      document.getElementById('laneBar').style.width = Math.min(Math.max((d.lane_error+0.5)/1.0*100,0),100)+'%';
      document.getElementById('cmdLinX').textContent = d.cmd_lin_x.toFixed(3);
      document.getElementById('cmdAngZ').textContent = d.cmd_ang_z.toFixed(3);

      // Odom
      document.getElementById('odomDist').textContent = d.distance.toFixed(2);
      document.getElementById('odomSpeed').textContent = d.speed.toFixed(3)+' m/s';

      // Speed & Selector
      document.getElementById('speedPct').textContent = d.speed_pct+'%';
      document.getElementById('speedBar').style.width = d.speed_pct+'%';
      // Update gear dots
      const gears = [25,40,60,100];
      gears.forEach((g,i) => {
        const dot = document.getElementById('gear'+i);
        if(dot) dot.classList.toggle('active', d.speed_pct >= g);
      });
      document.getElementById('ctrlState').textContent = d.ctrl_state_name;

      // Controller buttons — Updated to match user's specific mapping:
      // A=0, B=1, X=3, Y=4, LB=6, RB=7, Start=11
      const bm={0:'btnA',1:'btnB',3:'btnX',4:'btnY',6:'btnLB',7:'btnRB',8:'btnLT',9:'btnRT',11:'btnStart'};
      Object.values(bm).forEach(id=>document.getElementById(id).classList.remove('active'));
      if(d.buttons){
        d.buttons.forEach((v,i)=>{if(v&&bm[i])document.getElementById(bm[i]).classList.add('active');});
        const p=[];d.buttons.forEach((v,i)=>{if(v)p.push('btn['+i+']');});
        const db=document.getElementById('btnDebug');
        if(p.length){db.textContent='Active: '+p.join(', ');db.style.color='var(--success)';}
        else{db.textContent='No buttons pressed';db.style.color='var(--muted)';}
      }

      // D-pad (axes 6,7)
      if(d.axes&&d.axes.length>7){
        document.getElementById('btnLeft').classList.toggle('active',d.axes[6]>0.5);
        document.getElementById('btnRight').classList.toggle('active',d.axes[6]<-0.5);
        document.getElementById('btnUp').classList.toggle('active',d.axes[7]>0.5);
        document.getElementById('btnDown').classList.toggle('active',d.axes[7]<-0.5);
      }
      if(d.axes&&d.axes.length>3){
        document.getElementById('joyValL').textContent='L: '+d.axes[0].toFixed(1)+', '+d.axes[1].toFixed(1);
        document.getElementById('joyValR').textContent='R: '+d.axes[2].toFixed(1)+', '+d.axes[3].toFixed(1);
        
        let lx = 50 + (d.axes[0] * -50);
        let ly = 50 + (d.axes[1] * -50);
        let rx = 50 + (d.axes[2] * -50);
        let ry = 50 + (d.axes[3] * -50);
        
        const dl = document.getElementById('joyDotL');
        const dr = document.getElementById('joyDotR');
        
        dl.style.left = lx + '%';
        dl.style.top = ly + '%';
        dl.classList.toggle('active', Math.abs(d.axes[0]) > 0.05 || Math.abs(d.axes[1]) > 0.05);
        
        dr.style.left = rx + '%';
        dr.style.top = ry + '%';
        dr.classList.toggle('active', Math.abs(d.axes[2]) > 0.05 || Math.abs(d.axes[3]) > 0.05);
      }
    })
    .catch(()=>{
      document.getElementById('connDot').style.background='#f44336';
      document.getElementById('connText').textContent='Disconnected';
    });
}
// ===== PARAMETER TUNING (curated) =====
const PARAM_TIPS = {
  // Traffic Light
  red_h_low1:'Red hue range 1 lower bound (HSV)',  red_h_high1:'Red hue range 1 upper bound',
  red_h_low2:'Red hue range 2 lower bound (wrap)',  red_h_high2:'Red hue range 2 upper bound',
  yellow_h_low:'Yellow hue lower',  yellow_h_high:'Yellow hue upper',
  green_h_low:'Green hue lower',  green_h_high:'Green hue upper',
  sat_min:'Min saturation to count as color',  val_min:'Min brightness to count as color',
  min_circle_radius:'Smallest circle to detect',  max_circle_radius:'Largest circle to detect',
  min_pixel_count:'Min colored pixels to trigger',
  // Line Follower
  smoothing_alpha:'Error smoothing (0=smooth, 1=raw)',  dead_zone:'Ignore error below this',
  white_threshold:'Brightness to count as white line',  crop_ratio:'How far down to crop (0=top, 1=bottom)',
  // Auto Driver
  steering_gain:'How aggressively to steer on error',  forward_speed:'Base forward speed (m/s)',
  stale_timeout:'Seconds before data is considered stale',
  dist_obstruction_clear:'Distance after obstruction clears',  dist_roundabout:'Distance through roundabout',
  dist_boom_gate_1_pass:'Distance past boom gate 1',  dist_boom_gate_2_pass:'Distance past boom gate 2',
  dist_hill:'Distance over the hill',  dist_bumper:'Distance over bumpers',
  dist_traffic_light_pass:'Distance past green light',  dist_drive_to_perp:'Distance parallel → perp parking',
  // Boom Gate
  min_detect_dist:'Closest gate detection (m)',  max_detect_dist:'Farthest gate detection (m)',
  angle_window:'Forward arc width (rad)',  min_gate_points:'Min LiDAR points for gate',
  distance_variance_max:'Max spread of gate points',  lidar_angle_offset:'LiDAR mount rotation (rad)',
  // Obstruction
  detect_dist:'Start dodging at this distance (m)',  clear_dist:'Consider clear beyond this (m)',
  front_angle:'Front detection arc (rad)',  side_angle_min:'Side arc start (rad)',  side_angle_max:'Side arc end (rad)',
  steer_speed:'Speed while steering around (m/s)',  steer_angular:'Turn rate while dodging (rad/s)',
  pass_speed:'Speed while passing alongside (m/s)',  pass_duration:'Seconds driving alongside',
  steer_back_duration:'Seconds steering back to lane',  steer_away_duration:'Seconds steering away',
  // Parking
  parallel_forward_dist:'Drive past slot distance (m)',  parallel_reverse_dist:'Reverse into slot distance (m)',
  parallel_steer_angle:'Steering rate during reverse (rad/s)',  perp_turn_angle:'Turn angle into slot (rad)',
  perp_forward_dist:'Drive into slot distance (m)',  park_wait_time:'Seconds to wait in slot',
  drive_speed:'Parking drive speed (m/s)',  reverse_speed:'Parking reverse speed (m/s)',
  // Camera Obstacle
  edge_threshold:'Edge pixel ratio to trigger (0.0–1.0)',  canny_low:'Canny lower threshold',
  canny_high:'Canny upper threshold',  blur_kernel:'Gaussian blur kernel (odd number)',
  hysteresis_on:'Frames to confirm obstacle',  hysteresis_off:'Frames to confirm clear',
  show_debug:'Publish annotated debug frame',
};
const PARAM_GROUPS = [
  { node: 'traffic_light_detector', label: '🚦 Traffic Light', params: [
    'red_h_low1','red_h_high1','red_h_low2','red_h_high2',
    'yellow_h_low','yellow_h_high','green_h_low','green_h_high',
    'sat_min','val_min','min_circle_radius','max_circle_radius','min_pixel_count',
    'show_debug'
  ]},
  { node: 'line_follower_camera', label: '📐 Line Follower', params: [
    'smoothing_alpha','dead_zone','white_threshold','crop_ratio','show_debug'
  ]},
  { node: 'auto_driver', label: '🚗 Auto Driver', params: [
    'steering_gain','forward_speed','stale_timeout',
    'dist_obstruction_clear','dist_roundabout','dist_boom_gate_1_pass',
    'dist_boom_gate_2_pass','dist_hill','dist_bumper',
    'dist_traffic_light_pass','dist_drive_to_perp'
  ]},
  { node: 'boom_gate_detector', label: '🚧 Boom Gate', params: [
    'min_detect_dist','max_detect_dist','angle_window',
    'min_gate_points','distance_variance_max','lidar_angle_offset'
  ]},
  { node: 'obstruction_avoidance', label: '🔀 Obstruction', params: [
    'detect_dist','clear_dist','front_angle','side_angle_min','side_angle_max',
    'steer_speed','steer_angular','pass_speed','pass_duration',
    'steer_back_duration','lidar_angle_offset'
  ]},
  { node: 'parking_controller', label: '🅿️ Parking', params: [
    'parallel_forward_dist','parallel_reverse_dist','parallel_steer_angle',
    'perp_turn_angle','perp_forward_dist','park_wait_time',
    'drive_speed','reverse_speed'
  ]},
  { node: 'obstacle_avoidance_camera', label: '📷 Camera Obstacle', params: [
    'edge_threshold','canny_low','canny_high','blur_kernel',
    'hysteresis_on','hysteresis_off', 'show_debug'
  ]},
];

function buildParamUI() {
  const c = document.getElementById('paramContainer');
  c.innerHTML = PARAM_GROUPS.map(g => {
    const rows = g.params.map(p => {
      const tip = PARAM_TIPS[p] || '';
      return `<div class="param-row">
        <span class="param-name" ${tip ? 'title="'+tip+'"' : ''}>${p}</span>
        <input class="param-val" id="pv_${g.node}_${p}" placeholder="—" />
        <button class="param-get-btn" onclick="getParam('${g.node}','${p}')">Get</button>
        <button class="param-set-btn" onclick="setParam('${g.node}','${p}')">Set</button>
        <span class="param-status" id="ps_${g.node}_${p}"></span>
      </div>`;
    }).join('');
    return `<div class="param-node-block">
      <div class="param-node-header" onclick="this.nextElementSibling.classList.toggle('open')">
        <span class="param-node-name">${g.label}</span>
        <span class="param-node-count">${g.params.length} params</span>
      </div>
      <div class="param-node-body">${rows}</div>
    </div>`;
  }).join('');
}

async function getParam(node, param, isInitialLoad=false) {
  const status = document.getElementById('ps_' + node + '_' + param);
  const input = document.getElementById('pv_' + node + '_' + param);
  if (status && !isInitialLoad) { status.className = 'param-status'; status.textContent = '...'; }
  try {
    const r = await fetch('/api/get_param?node=' + encodeURIComponent(node) + '&param=' + encodeURIComponent(param));
    const d = await r.json();
    if (d.ok) {
      if (input) {
        input.value = d.value;
        if (isInitialLoad) {
          input.placeholder = `Def: ${d.value}`;
          const pName = input.parentElement.querySelector('.param-name');
          if (pName) {
            pName.innerHTML = `${param} <span style="color:#555;font-size:0.85em;">(def: ${d.value})</span>`;
          }
        }
      }
      if (status && !isInitialLoad) { status.className = 'param-status ok'; status.textContent = '✓'; }
    } else {
      if (status && !isInitialLoad) { status.className = 'param-status err'; status.textContent = d.error || 'Not found'; }
    }
  } catch(e) {
    if (status && !isInitialLoad) { status.className = 'param-status err'; status.textContent = 'Error'; }
  }
  if (!isInitialLoad) {
    setTimeout(() => { if(status) status.textContent = ''; }, 3000);
  }
}

async function setParam(node, param) {
  const input = document.getElementById('pv_' + node + '_' + param);
  const status = document.getElementById('ps_' + node + '_' + param);
  if (!input || !input.value) { if(status){status.className='param-status err';status.textContent='Empty';} return; }
  if (status) { status.className = 'param-status'; status.textContent = '...'; }
  try {
    const r = await fetch('/api/set_param', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({node: node, param: param, value: input.value})
    });
    const d = await r.json();
    if (d.ok) {
      if (status) { status.className = 'param-status ok'; status.textContent = '✓ Set'; }
    } else {
      if (status) { status.className = 'param-status err'; status.textContent = d.error || 'Failed'; }
    }
  } catch(e) {
    if (status) { status.className = 'param-status err'; status.textContent = 'Error'; }
  }
  setTimeout(() => { if(status) status.textContent = ''; }, 3000);
}

buildParamUI();

// Auto-fetch parameters on load to populate current values and defaults
setTimeout(async () => {
  for (const g of PARAM_GROUPS) {
    for (const p of g.params) {
      await getParam(g.node, p, true);
    }
  }
}, 1000);

setInterval(update, 200);
update();
</script>
</body>
</html>"""


# ======================== TEACH HTML Dashboard ========================
TEACH_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RISA-Bot Odometry &amp; Line Following</title>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;700;800&family=Inter:wght@400;700;800&display=swap" rel="stylesheet">
<style>
  :root {
    --bg: #0f0f13;
    --surface: #1e1e24;
    --text: #e6e9ef;
    --accent: #42a5f5;
    --success: #69f0ae;
    --danger: #ff5252;
    --warning: #ffd740;
  }
  * { margin:0; padding:0; box-sizing:border-box; }
  body {
    font-family: 'Inter', sans-serif;
    background: var(--bg);
    color: var(--text);
    height: 100vh;
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }
  header {
    padding: 20px 40px;
    background: var(--surface);
    display: flex;
    justify-content: space-between;
    align-items: center;
    border-bottom: 2px solid rgba(255,255,255,0.05);
  }
  header h1 { font-size: 2.2em; font-weight: 800; letter-spacing: -1px; }
  .badge { padding: 8px 16px; border-radius: 8px; font-weight: 700; font-size: 1.2em; }
  .badge.auto { background: rgba(105, 240, 174, 0.2); color: var(--success); }
  .badge.manual { background: rgba(255, 82, 82, 0.2); color: var(--danger); }
  
  main {
    flex: 1;
    display: flex;
    padding: 40px;
    gap: 40px;
  }
  .cam-panel {
    flex: 2;
    background: #000;
    border-radius: 20px;
    border: 2px solid rgba(255,255,255,0.1);
    overflow: hidden;
    position: relative;
    box-shadow: 0 20px 50px rgba(0,0,0,0.5);
    display: flex;
    flex-direction: column;
  }
  .cam-container {
    flex: 1;
    display: flex;
    justify-content: center;
    align-items: center;
    background: #050508;
  }
  .cam-container img {
    width: 100%;
    height: 100%;
    object-fit: contain;
  }
  
  .data-panel {
    flex: 1;
    display: flex;
    flex-direction: column;
    gap: 30px;
  }
  .data-card {
    background: var(--surface);
    border-radius: 20px;
    padding: 30px;
    border: 1px solid rgba(255,255,255,0.05);
    display: flex;
    flex-direction: column;
    justify-content: center;
  }
  .data-card h2 {
    font-size: 1.2em;
    text-transform: uppercase;
    letter-spacing: 3px;
    color: #888;
    margin-bottom: 10px;
  }
  .data-value {
    font-family: 'JetBrains Mono', monospace;
    font-size: 4.5em;
    font-weight: 800;
    line-height: 1.1;
  }
  .data-unit {
    font-size: 0.3em;
    color: #aaa;
    font-weight: 400;
    margin-left: 10px;
  }
  .odom-logs {
    margin-top: auto;
    background: rgba(0,0,0,0.3);
    padding: 15px;
    border-radius: 10px;
    font-family: 'JetBrains Mono', monospace;
    font-size: 0.9em;
    color: #aaa;
    border: 1px dashed rgba(255,255,255,0.1);
  }
  .odom-logs strong { color: #fff; margin-right: 10px; }
  .val-blue { color: var(--accent); }
  .val-green { color: var(--success); }
  .val-yellow { color: var(--warning); }
  .val-red { color: var(--danger); }
  
  /* Selectors */
  .cam-controls {
    display: flex; gap: 10px; padding: 15px; background: rgba(255,255,255,0.05);
  }
  .cam-btn {
    flex: 1; padding: 12px; border-radius: 10px; font-size: 1.1em; font-weight: 700;
    background: rgba(0,0,0,0.3); border: 1px solid rgba(255,255,255,0.1); color: #aaa;
    cursor: pointer; transition: all 0.2s;
  }
  .cam-btn.active {
    background: var(--accent); color: #fff; border-color: var(--accent);
  }
</style>
</head>
<body>

<header>
  <h1>RISA-Bot / Odometry &amp; Line Following</h1>
  <div id="modeBadge" class="badge">WAITING</div>
</header>

<main>
  <div class="cam-panel">
    <div class="cam-container">
      <img id="camStream" src="/camera_feed?v=line_follower" alt="Camera Feed Offline" onerror="this.style.display='none'; document.getElementById('camOff').style.display='flex';" onload="this.style.display='block'; document.getElementById('camOff').style.display='none';"/>
      <div id="camOff" style="display:none; color:#555; width:100%; height:100%; align-items:center; justify-content:center; flex-direction:column; font-size:1.5em; font-weight:700;">
        <div>∅ NO SIGNAL</div>
      </div>
    </div>
    <div class="cam-controls">
      <button class="cam-btn" onclick="setCam('raw')" id="btn-raw">Raw Feed</button>
      <button class="cam-btn active" onclick="setCam('line_follower')" id="btn-line_follower">Curve Tracking</button>
      <button class="cam-btn" onclick="setCam('obstacle')" id="btn-obstacle">Obstacle Edge</button>
    </div>
  </div>
  
  <div class="data-panel">
    <div class="data-card">
      <h2>Odometry / Distance</h2>
      <div class="data-value val-blue"><span id="odomDist">0.00</span><span class="data-unit">meters</span></div>
    </div>
    <div class="data-card">
      <h2>Velocity / Speed</h2>
      <div class="data-value val-green"><span id="odomSpeed">0.000</span><span class="data-unit">m/s</span></div>
    </div>
    <div class="data-card">
      <h2>Steering Error</h2>
      <div class="data-value val-yellow" id="steerValBlock"><span id="steerErr">0.000</span><span class="data-unit">ratio</span></div>
    </div>
    
    <div class="odom-logs">
      <div style="text-align:center; margin-bottom:10px; color:#555; font-size: 0.8em; font-weight:700;">RAW ODOMETRY LOGS</div>
      <div><strong>X:</strong> <span id="logX">0.000</span></div>
      <div><strong>Y:</strong> <span id="logY">0.000</span></div>
      <div><strong>Yaw:</strong> <span id="logYaw">0.000</span> rad</div>
    </div>
  </div>
</main>

<script>
function setCam(viewName) {
  fetch('/api/set_cam_view?view=' + viewName);
  document.getElementById('camStream').src = '/camera_feed?v=' + viewName + '&t=' + Date.now();
  ['raw', 'line_follower', 'obstacle'].forEach(v => {
    document.getElementById('btn-' + v).classList.toggle('active', v === viewName);
  });
}

// Default to line follower debug on page load
setCam('line_follower');

function update() {
  fetch('/data')
    .then(r => r.json())
    .then(d => {
      // Mode
      const mb = document.getElementById('modeBadge');
      mb.textContent = d.auto_mode ? 'AUTO MODE' : 'MANUAL MODE';
      mb.className = 'badge ' + (d.auto_mode ? 'auto' : 'manual');
      
      // Odom
      document.getElementById('odomDist').textContent = d.distance.toFixed(2);
      document.getElementById('odomSpeed').textContent = d.speed.toFixed(3);
      
      // Steering
      const err = d.lane_error;
      document.getElementById('steerErr').textContent = err > 0 ? '+' + err.toFixed(3) : err.toFixed(3);
      
      const sb = document.getElementById('steerValBlock');
      if (Math.abs(err) > 0.3) {
        sb.className = 'data-value val-red';
      } else if (Math.abs(err) > 0.1) {
        sb.className = 'data-value val-yellow';
      } else {
        sb.className = 'data-value val-green';
      }
      
      // Raw Logs
      document.getElementById('logX').textContent = (d.odom_x || 0).toFixed(3);
      document.getElementById('logY').textContent = (d.odom_y || 0).toFixed(3);
      document.getElementById('logYaw').textContent = (d.odom_yaw || 0).toFixed(3);
    })
    .catch(()=>{});
}

setInterval(update, 100);
update();
</script>
</body>
</html>"""

