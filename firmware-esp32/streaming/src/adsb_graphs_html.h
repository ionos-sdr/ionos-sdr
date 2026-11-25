/* adsb_graphs_html.h - graphs1090-style statistics page served on "/graphs".
 * Chart.js from cdnjs; data from data/history.json (one record per minute, 24 h). */
#pragma once
static const char ADSB_GRAPHS_HTML[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ionos-mini ADS-B graphs</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
<style>
body{margin:0;background:#15171a;color:#ddd;font:13px system-ui,sans-serif}
header{padding:10px 14px;border-bottom:1px solid #333;display:flex;gap:12px;align-items:center;flex-wrap:wrap}
header b{font-size:16px;color:#fff}a{color:#8ab4f8}
button{background:#23272e;color:#ddd;border:1px solid #444;border-radius:4px;padding:4px 10px;cursor:pointer}
button.on{background:#3a4a6a;border-color:#6a8acf;color:#fff}
#grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(560px,1fr));gap:12px;padding:12px}
.card{background:#1c1f24;border:1px solid #2c3038;border-radius:6px;padding:8px 10px}
.card h3{margin:0 0 6px;font-size:13px;color:#bbb;font-weight:600}
.card canvas{width:100%!important;height:220px!important}
#sum{color:#999}
@media (max-width:620px){#grid{grid-template-columns:1fr}}
</style></head><body>
<header><b>ionos-mini · FG23 ADS-B · statistics</b>
<span><button data-m="60">1 hour</button> <button data-m="360">6 hours</button> <button data-m="1440" class="on">24 hours</button> <button data-m="10080">7 days</button> <button data-m="43200">30 days</button> <button data-m="129600">90 days</button> <button data-m="525600">1 year</button></span>
<a id="ml" href="./">map</a><span id="sum"></span></header>
<div id="grid">
<div class="card"><h3>Messages / min</h3><canvas id="c1"></canvas></div>
<div class="card"><h3>Aircraft (last minute)</h3><canvas id="c2"></canvas></div>
<div class="card"><h3>Signal level (FG23 soft level − 30, approx. dBFS)</h3><canvas id="c3"></canvas></div>
<div class="card"><h3>Range (km, max per minute, from the reference point)</h3><canvas id="c4"></canvas></div>
<div class="card"><h3>Noise level (FG23 RSSI, dBm)</h3><canvas id="c5"></canvas></div>
<div class="card"><h3>FG23 load (triggers, preambles, FIFO full / min)</h3><canvas id="c6"></canvas></div>
<div class="card"><h3>Processed sample rate (MS/s, nominal 6.5)</h3><canvas id="c7"></canvas></div>
<div class="card"><h3>New tracks / min</h3><canvas id="c8"></canvas></div>
<div class="card"><h3>Aircraft with position: this receiver vs <a href="https://adsb.fi" target="_blank">adsb.fi</a> (250 NM)</h3><canvas id="c9"></canvas></div>
<div class="card"><h3>Coverage vs <a href="https://adsb.fi" target="_blank">adsb.fi</a> by distance (% of their aircraft also seen here)</h3><canvas id="c10"></canvas></div>
</div>
<p style="padding:0 14px 14px;color:#777">Reference data: <a href="https://adsb.fi" target="_blank">adsb.fi</a> open data (personal, non-commercial use).</p>
<script>
const q=location.search; document.getElementById('ml').href='./'+q;
Chart.defaults.color='#aaa'; Chart.defaults.borderColor='#2c3038'; Chart.defaults.font.size=11;
const C={};
function mk(id,sets,opt={}){const ctx=document.getElementById(id);
 C[id]=new Chart(ctx,{type:'line',data:{labels:[],datasets:sets.map(s=>({label:s[0],borderColor:s[1],backgroundColor:s[1]+'33',fill:!!s[2],
  pointRadius:0,borderWidth:1.5,tension:.2,spanGaps:true,data:[]}))},
  options:{animation:false,responsive:true,maintainAspectRatio:false,interaction:{mode:'index',intersect:false},
  plugins:{legend:{labels:{boxWidth:10}}},scales:{x:{ticks:{maxTicksLimit:8}},y:Object.assign({beginAtZero:true},opt.y||{})}}})}
mk('c1',[['all','#4fa3ff',1],['position','#ffb04f',0],['DF17','#7ee787',0]]);
mk('c2',[['aircraft','#4fa3ff',1],['with position','#ffb04f',1]]);
mk('c3',[['mean','#7ee787',0],['peak','#ff6b6b',0]],{y:{beginAtZero:false}});
mk('c4',[['max range','#d2a8ff',1]]);
mk('c5',[['noise','#999',0]],{y:{beginAtZero:false}});
mk('c6',[['trigger','#6e7681',0],['preamble','#4fa3ff',0],['FIFO full','#ff6b6b',0]]);
mk('c7',[['MS/s','#7ee787',1]],{y:{beginAtZero:true,suggestedMax:7}});
mk('c8',[['new aircraft','#ffb04f',1]]);
mk('c9',[['adsb.fi','#999',1],['this receiver','#4fa3ff',1],['both','#7ee787',0],['only here','#ffb04f',0]]);
mk('c10',[['0-50 km','#7ee787',0],['50-100 km','#4fa3ff',0],['100-200 km','#d2a8ff',0],['200-300 km','#ffb04f',0],['300+ km','#ff6b6b',0]],{y:{beginAtZero:true,max:100}});
let mins=1440;
function lab(t){const d=new Date(t*1000);return mins>1440?d.toLocaleDateString([], {month:'short',day:'numeric'})+' '+d.toLocaleTimeString([], {hour:'2-digit',minute:'2-digit'}):d.toLocaleTimeString([], {hour:'2-digit',minute:'2-digit'})}
function span(h){const m=h.n*(h.step||60)/60; return m>=2880?(m/1440).toFixed(1)+' days':m>=120?(m/60).toFixed(1)+' h':m+' min'}
function set(id,labels,...arrs){const c=C[id];c.data.labels=labels;arrs.forEach((a,i)=>c.data.datasets[i].data=a);c.update()}
async function load(){let h;try{h=await (await fetch('data/history.json'+q+(q?'&':'?')+'mins='+mins,{cache:'no-store'})).json()}catch(e){document.getElementById('sum').textContent='no connection';return}
 const L=h.t.map(lab);
 set('c1',L,h.msgs,h.pos,h.df17); set('c2',L,h.ac,h.acpos); set('c3',L,h.sig,h.peak); set('c4',L,h.range_km);
 set('c5',L,h.noise); set('c6',L,h.trig,h.pre,h.ovf); set('c7',L,h.msps); set('c8',L,h.tracks);
 const tot=Math.round(h.msgs.reduce((a,b)=>a+(b||0),0)*(h.step||60)/60), mx=Math.max(0,...h.range_km.filter(x=>x!=null));
 try{const b=await (await fetch('data/bench.json'+q+(q?'&':'?')+'mins='+mins,{cache:'no-store'})).json();const LB=b.t.map(lab);
  set('c9',LB,b.ref,b.ours,b.matched,b.only_ours); set('c10',LB,b.cov0,b.cov1,b.cov2,b.cov3,b.cov4);
  if(b.n){const r=b.ref.reduce((a,c)=>a+c,0),m=b.matched.reduce((a,c)=>a+c,0); var bs=` · coverage vs adsb.fi ${r?(100*m/r).toFixed(0):0} %`}}catch(e){}
 document.getElementById('sum').textContent=`${span(h)} of data`+(h.step>60?` (${h.step/60}-min averages)`:'')+(h.archive_since?` · archive since ${new Date(h.archive_since*1000).toLocaleDateString()}`:'')+` · ${tot} messages · max range ${mx.toFixed(0)} km`+(typeof bs!=='undefined'?bs:'')}
document.querySelectorAll('button[data-m]').forEach(b=>b.onclick=()=>{document.querySelectorAll('button[data-m]').forEach(x=>x.classList.remove('on'));b.classList.add('on');mins=+b.dataset.m;load()});
load(); setInterval(load,60000);
</script></body></html>)HTML";
