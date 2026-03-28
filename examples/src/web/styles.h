/*
 * styles.h — Web dashboard CSS for UniversalMesh Coordinator
 *
 * Included into the PROGMEM HTML string in web.cpp via C++ adjacent-string-literal
 * concatenation. Keeping styles here lets you edit them without touching web.cpp.
 *
 * Style: MMDVM-inspired (Arial, normal font sizes, MMDVM dark colour palette)
 */

#ifndef WEB_STYLES_H
#define WEB_STYLES_H

#define WEB_CSS \
"<style>" \
"*{box-sizing:border-box;margin:0;padding:0}" \
/* --- Light theme variables --- */ \
":root{" \
  "--bg:#f0f0f0;--container-bg:#fff;--text:#333;--muted:#6c757d;--sub:#6c757d;" \
  "--border:#dee2e6;--card-bg:#f8f9fa;--row-border:#dee2e6;" \
  "--topnav-bg:#333;--topnav-text:#f2f2f2;--topnav-hover:#555;" \
  "--dot-green:#28a745;" \
"}" \
/* --- Dark theme variables (MMDVM palette) --- */ \
"[data-theme=dark]{" \
  "--bg:#1a1a1a;--container-bg:#2d2d2d;--text:#fff;--muted:#adb5bd;--sub:#adb5bd;" \
  "--border:#555;--card-bg:#3a3a3a;--row-border:#444;" \
  "--topnav-bg:#000;--topnav-text:#f2f2f2;--topnav-hover:#444;" \
  "--dot-green:#28a745;" \
"}" \
/* --- Base --- */ \
"body{font-family:Arial,sans-serif;margin:0;padding-top:60px;background:var(--bg);color:var(--text);transition:background .3s,color .3s}" \
/* --- Fixed top navbar --- */ \
".navbar{position:fixed;top:0;left:0;right:0;background:var(--topnav-bg);border-bottom:1px solid var(--border);box-shadow:0 2px 5px rgba(0,0,0,0.3);z-index:1000;display:flex;align-items:center;padding:0 20px;height:60px}" \
".nav-brand{font-size:1.2em;font-weight:bold;color:var(--topnav-text);display:flex;align-items:center;gap:8px}" \
".nav-actions{margin-left:auto;display:flex;gap:6px;align-items:center}" \
".theme-btn{cursor:pointer;background:var(--topnav-hover);border:1px solid transparent;padding:8px 12px;border-radius:50%;font-size:1.1em;color:var(--topnav-text);line-height:1;transition:background .2s}" \
".theme-btn:hover{background:#007bff;color:white}" \
/* --- Layout --- */ \
".container{max-width:1000px;margin:20px auto;padding:0 16px}" \
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:15px;margin-bottom:15px}" \
/* --- Typography --- */ \
"h1{color:#007bff;border-bottom:2px solid #007bff;padding-bottom:10px;margin:0 0 20px 0;text-align:center;font-size:1.4em}" \
"h2{color:var(--muted);font-size:0.82em;text-transform:uppercase;letter-spacing:.1em;margin-bottom:10px;font-weight:bold}" \
/* --- Cards --- */ \
".card{background:var(--card-bg);border:1px solid var(--border);border-radius:6px;padding:20px}" \
/* --- Row / metric --- */ \
".row{display:flex;justify-content:space-between;align-items:center;padding:8px 0;border-bottom:1px solid var(--row-border)}" \
".row:last-child{border-bottom:none}" \
".lbl{color:var(--muted);font-weight:bold}" \
".val{color:var(--text)}" \
/* --- Tables --- */ \
"table{width:100%;border-collapse:collapse;margin-top:6px}" \
"th{color:var(--muted);text-align:left;padding:8px 6px;border-bottom:2px solid var(--border);font-weight:bold;white-space:nowrap}" \
"td{padding:8px 6px;border-bottom:1px solid var(--row-border);color:var(--text);word-break:break-all}" \
"#nodes-table td{word-break:normal;white-space:nowrap}" \
"#log-table td{word-break:normal;white-space:nowrap}" \
"tr:last-child td{border-bottom:none}" \
"tr:nth-child(even) td{background:rgba(0,0,0,0.06)}" \
"tr.relayed td:first-child{border-left:3px solid #2dd4ff;background:transparent!important;box-shadow:-2px 0 8px rgba(45,212,255,0.35),inset 1px 0 0 rgba(45,212,255,0.45)}" \
"[data-theme=dark] tr.relayed td:first-child{border-left:3px solid #2dd4ff;background:transparent!important;box-shadow:-2px 0 10px rgba(45,212,255,0.45),inset 1px 0 0 rgba(45,212,255,0.55)}" \
/* --- Buttons (MMDVM style: solid filled, semantic colours) --- */ \
".btn{display:inline-block;font-family:Arial,sans-serif;font-size:14px;font-weight:bold;padding:8px 16px;border:none;border-radius:4px;cursor:pointer;transition:background-color .3s ease;text-align:center;margin:2px}" \
".btn:hover:not(:disabled){opacity:.9}" \
".btn:disabled{opacity:.6;cursor:not-allowed;background-color:#ccc!important}" \
".btn-primary{background:#007bff;color:#fff}" \
".btn-primary:hover:not(:disabled){background:#0069d9}" \
".btn-success{background:#28a745;color:#fff}" \
".btn-success:hover:not(:disabled){background:#218838}" \
".btn-danger{background:#dc3545;color:#fff}" \
".btn-danger:hover:not(:disabled){background:#c82333}" \
".btn-secondary{background:#6c757d;color:#fff}" \
".btn-secondary:hover:not(:disabled){background:#545b62}" \
".action-buttons-vertical{display:flex;flex-direction:column;gap:10px;margin-top:15px}" \
".action-buttons-vertical .btn{width:100%;margin:0}" \
".sel{font-family:Arial,sans-serif;font-size:13px;background:var(--card-bg);color:var(--text);border:1px solid var(--border);border-radius:4px;padding:4px 8px}" \
/* --- Components --- */ \
".tag{font-size:12px;padding:3px 8px;border-radius:4px;background:#1293ff;color:#f4faff;font-weight:bold}" \
".dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--dot-green)}" \
".sub{color:var(--muted);font-size:0.85em;margin-top:6px}" \
".muted{color:var(--muted)}" \
".empty{color:var(--muted);font-size:0.9em;padding:10px 0}" \
".console-out{font-family:monospace;background:#111;color:#3fb950;height:220px;overflow-y:auto;white-space:pre-wrap;border-radius:4px;padding:10px;border:1px solid #333;font-size:13px}" \
/* --- Responsive --- */ \
"@media(max-width:600px){.grid{grid-template-columns:1fr}}" \
"</style>" \
/* Theme init: apply saved theme before first paint (avoids flash) */ \
"<script>(function(){var t=localStorage.getItem('theme')||'dark';document.documentElement.setAttribute('data-theme',t);})();</script>"

#endif // WEB_STYLES_H
