/* adsb_map_html.h - tar1090-style map page served by adsb_esp.cpp on "/".
 * Leaflet + OpenStreetMap from public CDNs; data from data/aircraft.json
 * (relative, query string passed through, so it also works behind a proxy). */
#pragma once
static const char ADSB_MAP_HTML[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
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
<div id="hdr"><b>ionos-mini · FG23 ADS-B</b> <a id="gl" href="graphs" style="color:#8ab4f8;margin-left:8px">graphs</a><br><span id="st" class="dim">…</span></div>
<table><thead><tr><th>ICAO</th><th>Callsign</th><th>Cat</th><th>Alt ft</th><th>GS kt</th><th>Track</th><th>Msgs</th><th>Seen</th><th>RSSI</th></tr></thead><tbody id="tb"></tbody></table>
</div></div>
<script>
const q=location.search, map=L.map('map',{worldCopyJump:true}).setView([47.5,19.05],7);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:12,attribution:'&copy; OpenStreetMap'}).addTo(map);
const ac={}, TR={}; let sel=null, hov=null, fitted=false, outline=null;
document.getElementById('gl').href='graphs'+q;
async function rng(){try{const o=await (await fetch('data/outline.json'+q,{cache:'no-store'})).json();
 const p=o.actualRange.last24h.points.map(x=>[x[0],x[1]]); if(p.length<3)return;
 if(!outline)outline=L.polygon(p,{color:'#bbb',weight:1.5,dashArray:'4 4',fill:true,fillColor:'#000',fillOpacity:.08,interactive:false}).addTo(map);
 else outline.setLatLngs(p)}catch(e){}}
function col(a){if(a==null)return'#999';const t=Math.max(0,Math.min(1,a/40000));return`hsl(${Math.round(30+t*270)},85%,55%)`}
const SZ=matchMedia('(pointer:coarse)').matches?34:28;
const SH={
 airliner:'M0,-15C1.3,-15 1.8,-13 1.8,-10L1.8,-3L14,4.5L14,6.5L1.8,3L1.3,10L5,12.6L5,14.3L0,13.2L-5,14.3L-5,12.6L-1.3,10L-1.8,3L-14,6.5L-14,4.5L-1.8,-3L-1.8,-10C-1.8,-13 -1.3,-15 0,-15ZM5.2,0.6h1.9v3.6h-1.9ZM-7.1,0.6h1.9v3.6h-1.9Z',
 heavy:'M0,-16C1.7,-16 2.3,-14 2.3,-10L2.3,-4L15.5,4L15.5,6.5L2.3,2.6L1.6,11L6,13.6L6,15.4L0,14.2L-6,15.4L-6,13.6L-1.6,11L-2.3,2.6L-15.5,6.5L-15.5,4L-2.3,-4L-2.3,-10C-2.3,-14 -1.7,-16 0,-16ZM5,-1h1.8v3.4h-1.8ZM-6.8,-1h1.8v3.4h-1.8ZM10,2h1.6v3h-1.6ZM-11.6,2h1.6v3h-1.6Z',
 smalljet:'M0,-13C1,-13 1.4,-11 1.4,-8L1.4,0L10,5L10,6.6L1.4,4L1.2,8L1.2,10L6,12L6,13.4L-6,13.4L-6,12L-1.2,10L-1.2,8L-1.4,4L-10,6.6L-10,5L-1.4,0L-1.4,-8C-1.4,-11 -1,-13 0,-13Z',
 light:'M-1.3,-12h2.6l-0.4,22h-1.8ZM-14,-7.5h28v3.2h-28ZM-5.5,8.5h11v2.6h-11ZM-2.6,-13.5h5.2v1.4h-5.2Z',
 fighter:'M0,-15L2,-6L12,7L12,9.5L2.2,7L3.4,12.5L-3.4,12.5L-2.2,7L-12,9.5L-12,7L-2,-6Z',
 heli:'M-2.2,-8C-2.2,-11 2.2,-11 2.2,-8L2.2,1C2.2,4 -2.2,4 -2.2,1ZM-0.7,2h1.4v11h-1.4ZM-3.5,11.5h7v1.6h-7Z',
 glider:'M-0.8,-10h1.6l-0.3,22h-1ZM-15.5,-3h31v1.8h-31ZM-4,10h8v1.6h-8Z',
 balloon:'M0,-13C7,-13 9,-6 7,-1C5.5,3 2,6 1.2,8h-2.4C-2,6 -5.5,3 -7,-1C-9,-6 -7,-13 0,-13ZM-2,10h4v3.5h-4Z',
 drone:'M-9,-9L9,9M9,-9L-9,9',
 generic:'M0,-13L8.5,11L0,6L-8.5,11Z'};
function shape(a){const c=a.category||'';
 const m={A1:'light',A2:'smalljet',A3:'airliner',A4:'airliner',A5:'heavy',A6:'fighter',A7:'heli',B1:'glider',B2:'balloon',B3:'light',B4:'light',B6:'drone'};
 if(m[c])return m[c];
 if((a.alt_baro||0)>20000||(a.gs||0)>300)return'airliner';
 if(a.alt_baro!=null&&a.alt_baro<6000&&a.gs!=null&&a.gs<140)return'light';
 return'generic'}
function icon(a,c){const s=shape(a), rot=(s=='balloon'||s=='drone')?0:(a.track||0);
 let g;
 if(s=='heli')g=`<circle r="12" fill="none" stroke="${c}" stroke-width="1.6" opacity=".85"/><path d="${SH.heli}" fill="${c}" stroke="#000" stroke-width=".7"/>`;
 else if(s=='drone')g=`<path d="${SH.drone}" stroke="${c}" stroke-width="2.4"/><circle cx="9" cy="9" r="3.4" fill="${c}" stroke="#000" stroke-width=".6"/><circle cx="-9" cy="9" r="3.4" fill="${c}" stroke="#000" stroke-width=".6"/><circle cx="9" cy="-9" r="3.4" fill="${c}" stroke="#000" stroke-width=".6"/><circle cx="-9" cy="-9" r="3.4" fill="${c}" stroke="#000" stroke-width=".6"/>`;
 else g=`<path d="${SH[s]}" fill="${c}" stroke="#000" stroke-width=".7" stroke-linejoin="round"/>`;
 return L.divIcon({className:'',iconSize:[SZ,SZ],iconAnchor:[SZ/2,SZ/2],
  html:`<svg width="${SZ}" height="${SZ}" viewBox="-16 -16 32 32" style="transform:rotate(${rot}deg);filter:drop-shadow(0 0 1px rgba(0,0,0,.6))">${g}</svg>`})}
/* track of the hovered / selected aircraft: history from data/trace.json, then extended live */
function seg(t,a,b){if(b[3]-a[3]>600)return; L.polyline([[a[0],a[1]],[b[0],b[1]]],{color:col(b[2]),weight:3,opacity:.95,interactive:false}).addTo(t.grp)}
function addPt(t,p){const l=t.pts[t.pts.length-1]; if(l&&l[0]==p[0]&&l[1]==p[1])return; t.pts.push(p); if(l)seg(t,l,p)}
async function showTrail(h){if(TR[h])return; const t=TR[h]={pts:[],ok:false,grp:L.layerGroup().addTo(map)};
 try{const r=await (await fetch('data/trace.json'+q+(q?'&':'?')+'hex='+h,{cache:'no-store'})).json();
  if(TR[h]!==t)return; for(const p of r.trace)addPt(t,[p[1],p[2],p[3],p[0]])}catch(e){}
 if(TR[h]!==t)return; t.ok=true; if(ac[h]){const ll=ac[h].getLatLng(); addPt(t,[ll.lat,ll.lng,ac[h].alt,Date.now()/1000])}}
function hideTrail(h){if(h==sel||h==hov||!TR[h])return; map.removeLayer(TR[h].grp); delete TR[h]}
function selectAc(h){const o=sel; sel=(o==h)?null:h; if(o)hideTrail(o); if(sel)showTrail(sel)}
map.on('click',()=>{const o=sel; sel=null; if(o)hideTrail(o)});
function fmt(v,d){return v==null?'':(d?v.toFixed(d):v)}
async function tick(){
 let j; try{j=await (await fetch('data/aircraft.json'+q,{cache:'no-store'})).json()}catch(e){document.getElementById('st').textContent='no connection';return}
 const seen=new Set(); let np=0;
 for(const a of j.aircraft){
  seen.add(a.hex); const c=col(a.alt_baro);
  if(a.lat!=null&&a.seen_pos<60){np++;
   const ll=[a.lat,a.lon], lab=(a.flight||a.hex).trim()+(a.alt_baro!=null?' '+Math.round(a.alt_baro/100):'');
   if(!ac[a.hex]){ac[a.hex]=L.marker(ll,{icon:icon(a,c)}).addTo(map).bindTooltip(lab,{permanent:true,direction:'right',offset:[SZ/2,0],className:'lbl'});
     const h=a.hex;
     ac[h].on('mouseover',()=>{hov=h;showTrail(h)}).on('mouseout',()=>{hov=null;hideTrail(h)})
      .on('click',()=>{selectAc(h);render(j)})}
   else{ac[a.hex].setLatLng(ll).setIcon(icon(a,c)).setTooltipContent(lab)}
   ac[a.hex].alt=a.alt_baro;
   const t=TR[a.hex]; if(t&&t.ok)addPt(t,[ll[0],ll[1],a.alt_baro,j.now]);
  } }
 for(const h in ac) if(!seen.has(h)){map.removeLayer(ac[h]);delete ac[h]; if(TR[h]){map.removeLayer(TR[h].grp);delete TR[h]}; if(sel==h)sel=null}
 if(!fitted&&np>2){const b=L.latLngBounds(Object.values(ac).map(m=>m.getLatLng()));map.fitBounds(b.pad(0.2));fitted=true}
 render(j,np)}
function render(j,np){
 const rows=j.aircraft.slice().sort((a,b)=>a.seen-b.seen).map(a=>`<tr data-h="${a.hex}" class="${a.hex==sel?'sel':''}"><td>${a.hex}</td><td>${(a.flight||'').trim()}</td><td>${a.category||''}</td><td>${fmt(a.alt_baro)}</td><td>${fmt(a.gs,0)}</td><td>${fmt(a.track,0)}</td><td>${a.messages}</td><td>${a.seen.toFixed(0)}s</td><td>${fmt(a.rssi,1)}</td></tr>`).join('');
 const tb=document.getElementById('tb'); tb.innerHTML=rows;
 tb.querySelectorAll('tr').forEach(r=>r.onclick=()=>{const h=r.dataset.h; if(sel!=h)selectAc(h); if(ac[h])map.panTo(ac[h].getLatLng()); render(j,np)});
 if(np!=null)document.getElementById('st').textContent=`${j.aircraft.length} aircraft, ${np} with position · ${j.messages} messages · ${new Date(j.now*1000).toLocaleTimeString()}`}
tick(); setInterval(tick,1000); rng(); setInterval(rng,30000);
</script></body></html>)HTML";
