/* adsb_map_html.h - tar1090-style map page served by adsb_esp.cpp on "/".
 * Leaflet + OpenStreetMap from public CDNs; data from data/aircraft.json
 * (relative, query string passed through, so it also works behind a proxy). */
#pragma once
static const char ADSB_MAP_HTML[] = R"HTML(<!doctype html>
<html lang="hu"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ionos-mini ADS-B</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<style>
html,body{margin:0;height:100%;font:13px system-ui,sans-serif;background:#111;color:#ddd}
#wrap{display:flex;height:100%}#map{flex:1}
#side{width:380px;overflow:auto;background:#1a1d22;border-left:1px solid #333}
#hdr{padding:8px 10px;border-bottom:1px solid #333}#hdr b{color:#fff;font-size:15px}
table{width:100%;border-collapse:collapse}th,td{padding:3px 6px;text-align:right;white-space:nowrap}
th{position:sticky;top:0;background:#23272e;color:#aaa;font-weight:600}td:first-child,th:first-child,td:nth-child(2),th:nth-child(2){text-align:left}
tr:nth-child(even){background:#1f2228}tr.sel{background:#3a4a6a}tr:hover{background:#2c3340;cursor:pointer}
.lbl{background:rgba(20,22,26,.8);color:#fff;border:0;border-radius:3px;padding:1px 4px;font:11px monospace;box-shadow:none}
.lbl:before{display:none}.dim{color:#888}
@media (max-width:700px){#wrap{flex-direction:column}#side{width:auto;height:40%}}
</style></head><body><div id="wrap"><div id="map"></div><div id="side">
<div id="hdr"><b>ionos-mini · FG23 ADS-B</b> <a id="gl" href="graphs" style="color:#8ab4f8;margin-left:8px">grafikonok</a><br><span id="st" class="dim">…</span></div>
<table><thead><tr><th>ICAO</th><th>Hívójel</th><th>Mag. ft</th><th>Seb. kt</th><th>Irány</th><th>Üz.</th><th>Látva</th><th>RSSI</th></tr></thead><tbody id="tb"></tbody></table>
</div></div>
<script>
const q=location.search, map=L.map('map',{worldCopyJump:true}).setView([47.5,19.05],7);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:12,attribution:'&copy; OpenStreetMap'}).addTo(map);
const ac={}, trails={}; let sel=null, fitted=false, outline=null;
document.getElementById('gl').href='graphs'+q;
async function rng(){try{const o=await (await fetch('data/outline.json'+q,{cache:'no-store'})).json();
 const p=o.actualRange.last24h.points.map(x=>[x[0],x[1]]); if(p.length<3)return;
 if(!outline)outline=L.polygon(p,{color:'#bbb',weight:1.5,dashArray:'4 4',fill:true,fillColor:'#000',fillOpacity:.08,interactive:false}).addTo(map);
 else outline.setLatLngs(p)}catch(e){}}
function col(a){if(a==null)return'#999';const t=Math.max(0,Math.min(1,a/40000));return`hsl(${Math.round(30+t*270)},85%,55%)`}
function icon(trk,c){return L.divIcon({className:'',iconSize:[22,22],iconAnchor:[11,11],html:
`<svg width="22" height="22" viewBox="-11 -11 22 22" style="transform:rotate(${trk||0}deg)"><path d="M0,-10 L2,-3 L9,1 L9,3 L2,1 L1.5,7 L4,9 L4,10 L0,9 L-4,10 L-4,9 L-1.5,7 L-2,1 L-9,3 L-9,1 L-2,-3 Z" fill="${c}" stroke="#000" stroke-width=".8"/></svg>`})}
function fmt(v,d){return v==null?'':(d?v.toFixed(d):v)}
async function tick(){
 let j; try{j=await (await fetch('data/aircraft.json'+q,{cache:'no-store'})).json()}catch(e){document.getElementById('st').textContent='nincs kapcsolat';return}
 const seen=new Set(); let np=0;
 for(const a of j.aircraft){
  seen.add(a.hex); const c=col(a.alt_baro);
  if(a.lat!=null&&a.seen_pos<60){np++;
   const ll=[a.lat,a.lon], lab=(a.flight||a.hex).trim()+(a.alt_baro!=null?' '+Math.round(a.alt_baro/100):'');
   if(!ac[a.hex]){ac[a.hex]=L.marker(ll,{icon:icon(a.track,c)}).addTo(map).bindTooltip(lab,{permanent:true,direction:'right',offset:[10,0],className:'lbl'});
     ac[a.hex].on('click',()=>{sel=a.hex;render(j)})}
   else{ac[a.hex].setLatLng(ll).setIcon(icon(a.track,c)).setTooltipContent(lab)}
   const t=trails[a.hex]||(trails[a.hex]={pts:[],line:L.polyline([],{color:c,weight:2,opacity:.7}).addTo(map)});
   const last=t.pts[t.pts.length-1]; if(!last||last[0]!=ll[0]||last[1]!=ll[1]){t.pts.push(ll); if(t.pts.length>200)t.pts.shift(); t.line.setLatLngs(t.pts).setStyle({color:c})}
  } }
 for(const h in ac) if(!seen.has(h)){map.removeLayer(ac[h]);delete ac[h]; if(trails[h]){map.removeLayer(trails[h].line);delete trails[h]}}
 if(!fitted&&np>2){const b=L.latLngBounds(Object.values(ac).map(m=>m.getLatLng()));map.fitBounds(b.pad(0.2));fitted=true}
 render(j,np)}
function render(j,np){
 const rows=j.aircraft.slice().sort((a,b)=>a.seen-b.seen).map(a=>`<tr data-h="${a.hex}" class="${a.hex==sel?'sel':''}"><td>${a.hex}</td><td>${(a.flight||'').trim()}</td><td>${fmt(a.alt_baro)}</td><td>${fmt(a.gs,0)}</td><td>${fmt(a.track,0)}</td><td>${a.messages}</td><td>${a.seen.toFixed(0)}s</td><td>${fmt(a.rssi,1)}</td></tr>`).join('');
 const tb=document.getElementById('tb'); tb.innerHTML=rows;
 tb.querySelectorAll('tr').forEach(r=>r.onclick=()=>{sel=r.dataset.h; if(ac[sel])map.panTo(ac[sel].getLatLng()); render(j,np)});
 if(np!=null)document.getElementById('st').textContent=`${j.aircraft.length} gép, ${np} pozícióval · ${j.messages} üzenet · ${new Date(j.now*1000).toLocaleTimeString()}`}
tick(); setInterval(tick,1000); rng(); setInterval(rng,30000);
</script></body></html>)HTML";
