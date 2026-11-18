import random, itertools, math, json
from pathlib import Path
from collections import defaultdict
# Baseline 10:15 model: 40 L = A 13.09 L, B 8.73 L, C 18.18 L. 9 missions.
HUBS=('A','B','C')
counts=(3,2,4)
loads={'A':(5.0,5.0,3.09),'B':(5.0,3.73),'C':(5.0,5.0,5.0,3.18)}
distance={'A':1.5,'B':2.5,'C':3.5}
flight_speed=30.0
travel={h: int(round(2*d/flight_speed*60)) for h,d in distance.items()} # 6,10,14
priority={'A':2,'B':1,'C':3}
deadline={'A':15,'B':25,'C':10}
LATE_PENALTY=35.0
MAKESPAN_PENALTY=0.5

def select_drone(avails): return 0 if avails[0]<=avails[1] else 1

def step(state, h):
    a,b,c,t1,t2=state; cnt=(a,b,c);i=HUBS.index(h)
    assert cnt[i]<counts[i]
    drone=select_drone((t1,t2));start=(t1,t2)[drone];arrival=start+travel[h]/2;ret=start+travel[h]
    tmp=list(cnt);tmp[i]+=1
    nxt=(tmp[0],tmp[1],tmp[2],ret if drone==0 else t1, ret if drone==1 else t2)
    first=cnt[i]==0
    late=max(0, arrival-deadline[h]) if first else 0
    reward= -priority[h]*arrival -LATE_PENALTY*late
    terminal=tuple(tmp)==counts
    if terminal: reward-=MAKESPAN_PENALTY*max(nxt[3],nxt[4])
    return nxt,reward,dict(hub=h,drone=f'UAV {drone+1}',start=start,arrival=arrival,finish=ret,load=loads[h][cnt[i]],first=first,deadline=deadline[h],late=late)

def simulate(seq):
    s=(0,0,0,0,0); rows=[];ret=0
    for h in seq:
        s,r,row=step(s,h);ret+=r;rows.append(row)
    first={h: next(x['arrival'] for x in rows if x['hub']==h) for h in HUBS}
    return dict(score=ret,first=first,missed=sum(first[h]>deadline[h] for h in HUBS),makespan=max(x['finish'] for x in rows),weighted_arrival=sum(priority[x['hub']]*x['arrival'] for x in rows),late=sum(max(0,first[h]-deadline[h]) for h in HUBS),rows=rows,sequence=''.join(seq))

def seq_order(order): return list(''.join(h*counts[HUBS.index(h)] for h in order))
# nearest and priority; approximate previous hybrid method: greedy with diversity penalizer

def hybrid_seq():
    s=(0,0,0,0,0); seq=[]
    while s[:3]!=counts:
        avail=[h for i,h in enumerate(HUBS) if s[i]<counts[i]]
        # 3*priority -0.3*roundtrip time - 2*previously scheduled trips
        h=max(avail,key=lambda h: (3*priority[h]-0.3*travel[h]-2*s[HUBS.index(h)],-HUBS.index(h)))
        seq.append(h); s,_,_=step(s,h)
    return seq
# deadline-aware rule: minimize risk of missing first deadline, then weighted balance
# best manually chosen example matching previous deadline strategy, without pretending optimum
seq_deadline=list('CAB')+list('CCCAAABB') # WRONG counts; replace below
seq_deadline=list('CABCAACCB') # deadline-aware reference: A first3, B first11, C first7; 52 min makespan
print('baseline seq candidates', ''.join(seq_deadline),{h:seq_deadline.count(h) for h in HUBS})
strategies={
 'Nearest-first':seq_order('ABC'),
 'Priority-first':seq_order('CAB'),
 'Hybrid (rule)':hybrid_seq(),
 'Deadline-aware (rule)':seq_deadline,
}
for name,seq in strategies.items():
    r=simulate(seq);print(name,r['sequence'],r['first'],r['missed'],r['makespan'],r['score'])
# exact exhaustive sequences for 3xA 2xB 4xC
best=(float('-inf'),None); n=0
for seq in set(itertools.permutations('AAABBCCCC')):
    n+=1;r=simulate(seq)
    if r['score']>best[0]:best=(r['score'],seq)
print('exhaustive',n,best[0],''.join(best[1]),simulate(best[1])['first'],simulate(best[1])['makespan'])
# tabular Q-learning: learn which HUB next, UAV chosen as earliest available (identical fleet)
def q_train(episodes=30000,seed=11,alpha=.25,gamma=1.0):
    rnd=random.Random(seed);Q=defaultdict(lambda:[0.0]*3); learning=[]
    for ep in range(episodes):
        s=(0,0,0,0,0)
        # decay exploration 1 -> .04
        eps=max(.04,0.95*(.99975**ep))
        for t in range(9):
            candidates=[i for i in range(3) if s[i]<counts[i]]
            if rnd.random()<eps:
                ai=rnd.choice(candidates)
            else:
                ai=max(candidates,key=lambda i:(Q[s][i],-i))
            s1,r,_=step(s,HUBS[ai]);nextc=[i for i in range(3) if s1[i]<counts[i]]
            y=r+gamma*max(Q[s1][j] for j in nextc) if nextc else r
            Q[s][ai]+=alpha*(y-Q[s][ai]);s=s1
        if (ep+1)%500==0:
            seq=q_eval(Q);res=simulate(seq);learning.append({'episode':ep+1,'score':res['score'],'missed':res['missed'],'makespan':res['makespan']})
    return Q,learning

def q_eval(Q):
    s=(0,0,0,0,0);seq=[]
    while s[:3]!=counts:
        cand=[i for i in range(3) if s[i]<counts[i]]
        ai=max(cand,key=lambda i:(Q[s][i],-i));h=HUBS[ai];seq.append(h);s,_,_=step(s,h)
    return seq
Q, learning=q_train()
qseq=q_eval(Q);qr=simulate(qseq)
print('Q-learning',qr['sequence'],qr['first'],qr['missed'],qr['makespan'],qr['score']);print('learning last:',learning[-10:])
for k,v in strategies.items():
    print(k,simulate(v)['sequence'])
# save reusable model results
out={'scenario':{'counts':counts,'loads':loads,'distance':distance,'travel':travel,'priority':priority,'deadline':deadline,'late_penalty':LATE_PENALTY,'makespan_penalty':MAKESPAN_PENALTY},'methods':{k:simulate(v) for k,v in strategies.items()},'QLearning':qr,'ExactSearchReference':simulate(best[1]),'learning':learning,'training':{'episodes':30000,'alpha':.25,'gamma':1.0,'epsilon_start':.95,'epsilon_min':.04,'epsilon_decay':.99975,'seed':11}, 'full_sequences':n}
with open(Path(__file__).resolve().parent / 'module04_ai_results.json', 'w', encoding='utf-8') as f:json.dump(out,f,indent=2)
