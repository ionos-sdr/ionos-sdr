/* adsb_graphs_html.h - graphs1090-style statistics page served on "/graphs".
 * Chart.js from cdnjs; data from data/history.json (one record per minute, 24 h). */
#pragma once
static const char ADSB_GRAPHS_HTML[] = R"HTML(<!doctype html>
<html lang="hu"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ionos-mini ADS-B grafikonok</title>
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
<header><b>ionos-mini · FG23 ADS-B · statisztika</b>
<span><button data-m="60">1 óra</button> <button data-m="360">6 óra</button> <button data-m="1440" class="on">24 óra</button></span>
<a id="ml" href="./">térkép</a><span id="sum"></span></header>
<div id="grid">
<div class="card"><h3>Üzenetek / perc</h3><canvas id="c1"></canvas></div>
<div class="card"><h3>Repülőgépek (az utolsó percben)</h3><canvas id="c2"></canvas></div>
<div class="card"><h3>Jelszint (FG23 soft szint − 30, kb. dBFS)</h3><canvas id="c3"></canvas></div>
<div class="card"><h3>Hatótáv (km, max / perc, a referenciaponttól)</h3><canvas id="c4"></canvas></div>
<div class="card"><h3>Zajszint (FG23 RSSI, dBm)</h3><canvas id="c5"></canvas></div>
<div class="card"><h3>FG23 terhelés (trigger, preamble, FIFO-telítés / perc)</h3><canvas id="c6"></canvas></div>
<div class="card"><h3>Feldolgozott mintasebesség (MS/s, névleges 6,5)</h3><canvas id="c7"></canvas></div>
<div class="card"><h3>Új nyomvonalak / perc</h3><canvas id="c8"></canvas></div>
</div>
<script>
const q=location.search; document.getElementById('ml').href='./'+q;
Chart.defaults.color='#aaa'; Chart.defaults.borderColor='#2c3038'; Chart.defaults.font.size=11;
const C={};
function mk(id,sets,opt={}){const ctx=document.getElementById(id);
 C[id]=new Chart(ctx,{type:'line',data:{labels:[],datasets:sets.map(s=>({label:s[0],borderColor:s[1],backgroundColor:s[1]+'33',fill:!!s[2],
  pointRadius:0,borderWidth:1.5,tension:.2,spanGaps:true,data:[]}))},
  options:{animation:false,responsive:true,maintainAspectRatio:false,interaction:{mode:'index',intersect:false},
  plugins:{legend:{labels:{boxWidth:10}}},scales:{x:{ticks:{maxTicksLimit:8}},y:Object.assign({beginAtZero:true},opt.y||{})}}})}
mk('c1',[['összes','#4fa3ff',1],['pozíció','#ffb04f',0],['DF17','#7ee787',0]]);
mk('c2',[['gépek','#4fa3ff',1],['pozícióval','#ffb04f',1]]);
mk('c3',[['átlag','#7ee787',0],['csúcs','#ff6b6b',0]],{y:{beginAtZero:false}});
mk('c4',[['max hatótáv','#d2a8ff',1]]);
mk('c5',[['zaj','#999',0]],{y:{beginAtZero:false}});
mk('c6',[['trigger','#6e7681',0],['preamble','#4fa3ff',0],['FIFO-telítés','#ff6b6b',0]]);
mk('c7',[['MS/s','#7ee787',1]],{y:{beginAtZero:true,suggestedMax:7}});
mk('c8',[['új gépek','#ffb04f',1]]);
let mins=1440;
function lab(t){const d=new Date(t*1000);return d.toLocaleTimeString([], {hour:'2-digit',minute:'2-digit'})}
function set(id,labels,...arrs){const c=C[id];c.data.labels=labels;arrs.forEach((a,i)=>c.data.datasets[i].data=a);c.update()}
async function load(){let h;try{h=await (await fetch('data/history.json'+q+(q?'&':'?')+'mins='+mins,{cache:'no-store'})).json()}catch(e){document.getElementById('sum').textContent='nincs kapcsolat';return}
 const L=h.t.map(lab);
 set('c1',L,h.msgs,h.pos,h.df17); set('c2',L,h.ac,h.acpos); set('c3',L,h.sig,h.peak); set('c4',L,h.range_km);
 set('c5',L,h.noise); set('c6',L,h.trig,h.pre,h.ovf); set('c7',L,h.msps); set('c8',L,h.tracks);
 const tot=h.msgs.reduce((a,b)=>a+b,0), mx=Math.max(0,...h.range_km.filter(x=>x!=null));
 document.getElementById('sum').textContent=`${h.n} perc adat · ${tot} üzenet · max hatótáv ${mx.toFixed(0)} km`}
document.querySelectorAll('button[data-m]').forEach(b=>b.onclick=()=>{document.querySelectorAll('button[data-m]').forEach(x=>x.classList.remove('on'));b.classList.add('on');mins=+b.dataset.m;load()});
load(); setInterval(load,60000);
</script></body></html>)HTML";
