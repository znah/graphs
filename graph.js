"use strict";
const NN=3;
const CaseN = (NN+1)*2;

const calcCases = (nodes, states)=>nodes.map((node, i)=>{
    const nc = node.reduce((a, j)=>a+states[j], 0);
    return nc + states[i]*(NN+1)
});

class GrowingGraph {
    constructor(rule=2182) {
        this.rule = rule;
        this.lastGen = 0;
        this.nodes = [[9,1,2],[0,2,4],[1,3,0],[2,4,6],[3,5,1],
                      [4,6,8],[5,7,3],[6,8,9],[7,9,5],[8,0,7]];
        this.states = [0,0,0,1,0, 1,0,1,1,1];
        this.dividing = [];
        this.phase = 0;
        this.nodes.forEach(n=>{n.gen = 0;});
    }

    reconnect(source, oldPeer, newPeer) {
        const node = this.nodes[source];
        node[node.indexOf(oldPeer)] = newPeer;
    }

    grow() {
        const {nodes, states, rule} = this;

        if (this.phase === 0) { // update states
            calcCases(nodes, states).forEach((r,i)=>{
                this.states[i] = (rule >> r) & 1;
                this.dividing[i] = (rule >> (r+CaseN)) & 1;
                // const noise = Math.random();
                // if (noise<0.00005) {
                //     this.states[i] = 1-this.states[i];
                // }
             });
        } else {
            ++this.lastGen;
            for (let i=0; i < this.dividing.length; ++i) {
                if (!this.dividing[i]) continue;
                const [a,b,c] = nodes[i];
                const j=nodes.length, k=j+1;
                Object.assign(nodes[i], [a,j,k]);
                nodes.push([i,b,k]);
                nodes.push([i,j,c]);
                states.push(states[i], states[i]);
                this.reconnect(b, i, j);
                this.reconnect(c, i, k);
                nodes[i].gen = nodes[j].gen = nodes[k].gen = this.lastGen;
                this.dividing[i] = 0;
            }
        }
        this.phase = 1-this.phase;
        return this;
    }
}
