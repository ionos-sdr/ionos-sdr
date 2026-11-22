import numpy as np, sys
from scipy.signal import firwin, lfilter
def analyse(fn, off=-10000):
    x=np.load(fn); fs=50000
    n=np.arange(len(x)); y=lfilter(firwin(65,6000/(fs/2)),1,x*np.exp(-2j*np.pi*off*n/fs))
    env=np.convolve(np.abs(y),np.ones(25)/25,'same'); fl=np.median(env)
    idx=np.flatnonzero(env>fl*4)
    if not len(idx): return 0,0
    br=np.split(idx,np.flatnonzero(np.diff(idx)>400)+1); nb=nh=0
    for b in br:
        if len(b)<600: continue
        nb+=1; s0,s1=b[0],b[-1]; lev=np.median(env[s0:s1])
        ins=env[s0:s1]<lev*0.35; d=np.diff(np.concatenate([[0],ins.astype(int),[0]]))
        L=np.flatnonzero(d==-1)-np.flatnonzero(d==1)
        if (L>=50).any(): nh+=1
    return nb,nh
for f in sys.argv[1:]:
    nb,nh=analyse(f); print("%s: bursts %d, with hole %d"%(f.split('/')[-1],nb,nh))
