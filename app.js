"use strict";

class App {
    constructor(canvas, options = {}) {
        this.canvas = canvas;
        this.glsl = SwissGL(canvas);
        this.options = Object.assign({
            autonomous: false,
            growthLimit: 10000,
            tickSteps: 2,
        }, options);

        this.graph = null;
        this.simulation = null;
        this.wasm = null;
        this.bloom = null;
        
        this.rules = [0x426, 0x8a2, 0x8ae, 0x886, 0x887, 0x8bc, 0x457, 0x26a, 0x409, 0x1016, 0x897, 0x4625, 0x4621, 0x6621, 0x56cc, 0xcbc, 0x3051, 0x1082, 0x289, 0x21f2];
        this.presets = {
            'quartatic'         : {rule: 2182,  flipProb: 0.0},
            'quartatic - mutations' : {rule: 2182,  flipProb: 5e-5},
            'two branches'      : {rule: 3260,  flipProb: 0.0},
            'exp tree'              : {rule: 2236, flipProb: 0.0},
            'exp hyper'   : {rule: 618, flipProb: 0.0},
            'exp fractal'   : {rule: 649, flipProb: 0.0},
            'exp symmethry'   : {rule: 1111, flipProb: 0.0},
            'rubust linear'     : {rule: 22220, flipProb: 1e-3},
            'stable explision'  : {rule: 8690,  flipProb: 1e-3},
            'fancy tentacles'   : {rule: 17953, flipProb: 5e-5},
        };

        this.params = {
            rule: 2182,
            flipProb: 0.0,
            dim: 2,
            preset: '-',
            reset: () => this.reset(),
        };

        this.gui = new dat.GUI();
        this.gui.add(this.params, 'rule', this.rules).onChange(() => this.reset());
        this.gui.add(this.params, 'flipProb', [0.0, 1e-3, 1e-4, 5e-5, 1e-5]).onChange(v => {
            if (this.graph) this.graph.flipProb = v;
        });
        this.gui.add(this.params, 'dim', [2, 3]).onChange(v => {
            if (this.simulation) this.simulation.dim = v;
        });
        this.gui.add(this.params, 'preset', Object.keys(this.presets)).onChange(name => {
            Object.assign(this.params, this.presets[name]);
            this.gui.updateDisplay();
            this.reset();
        });
        this.gui.add(this.params, 'reset');

        if (this.options.autonomous) {
            this.gui.close();
        }

        this.points = new Float32Array(256 * 256 * 4);
        this.links = new Int32Array(256 * 256 * 2);

        this.autoTimer = 0;
        this.prevNodeCount = 0;
        this.stallTimer = 0;
        this.transitionAlpha = 0;
        this.transitioningOut = false;
        this.consecutiveExplosions = 0;

        this.status = document.getElementById('status');
    }

    async init() {
        const response = await fetch('main.wasm');
        const buffer = await response.arrayBuffer();
        this.wasm = await WebAssembly.instantiate(buffer);
        this.bloom = new Bloom(this.glsl, this.gui);
        
        if (this.options.autonomous) {
            this.pickNewRule();
        } else {
            this.reset();
        }
        
        requestAnimationFrame((t) => this.frame(t));
    }

    reset() {
        this.graph = new GrowingGraph(this.params.rule);
        this.graph.flipProb = this.params.flipProb;
        this.simulation = new GraphLayout(this.graph, this.wasm.instance, this.params.dim);
        
        this.autoTimer = 0;
        this.stallTimer = 0;
        this.prevNodeCount = 0;
    }

    pickNewRule() {
        const presetKeys = Object.keys(this.presets);
        const r = Math.random();
        if (r < 0.1) {
            const randKey = presetKeys[Math.floor(Math.random() * presetKeys.length)];
            Object.assign(this.params, this.presets[randKey]);
            this.params.preset = randKey;
        } else if (r < 0.3) {
            this.params.rule = this.rules[Math.floor(Math.random() * this.rules.length)];
            this.params.flipProb = [0.0, 1e-3, 1e-4, 5e-5, 1e-5][Math.floor(Math.random() * 5)];
            this.params.preset = '-';
        } else {
            let lower = Math.floor(Math.random() * 256);
            let bit1 = Math.floor(Math.random() * 8);
            let bit2 = Math.floor(Math.random() * 8);
            let upper = (1 << bit1) | (1 << bit2);
            this.params.rule = (upper << 8) | lower;
            this.params.flipProb = [0.0, 1e-3, 1e-4, 5e-5, 1e-5][Math.floor(Math.random() * 5)];
            this.params.preset = '-';
        }
        this.gui.updateDisplay();
        this.reset();
        this.autoTimer = 0;
        this.stallTimer = 0;
        this.prevNodeCount = 0;
    }

    renderGraph(target = null) {
        if (!this.simulation || !this.graph) return target;
        
        const { treeCenter, treeExtent } = this.simulation;
        const center = treeCenter;
        const { pointN, pos } = this.simulation;
        const { lastGen } = this.graph;

        for (let i = 0; i < pointN; ++i) {
            for (let c = 0; c < 3; ++c) {
                this.points[i * 4 + c] = pos[i * 3 + c];
            }
            this.points[i * 4 + 3] = this.graph.nodes[i].gen;
        }
        
        const linkN = this.graph.links.length / 2;
        this.links.set(this.graph.links);
        
        const renderData = {
            pointsTex: this.glsl({}, { data: this.points, size: [256, 256], format: 'rgba32f', tag: 'points' }),
            linksTex: this.glsl({}, { data: this.links, size: [256, 256], format: 'rg32i', tag: 'links' }),
            center, extent: treeExtent, lastGen, Blend: 's*sa+d*(1-sa)', Aspect: 'fit'
        };

        this.glsl({ ...renderData, Grid: [linkN], VP: `
            #define id2xy(id) ivec2((id)&0xff, (id)>>8)
            ivec2 link = linksTex(id2xy(ID.x)).xy;
            vec3 p0 = pointsTex(id2xy(link.x)).xyz-center;
            vec3 p1 = pointsTex(id2xy(link.y)).xyz-center;
            vec2 dp = normalize(p1.xy-p0.xy);
            vec2 p = mix(p0, p1, UV.x).xy + vec2(-dp.y, dp.x)*XY.y;
            VPos.xy = 1.9*p.xy/(extent+30.);
        `, FP: `0.9` }, target);

        this.glsl({ ...renderData, Grid: [pointN], VP: `
            vec3 color_map(float t) {
                const vec3 c0 = vec3(0.06, 0.02, 0.54);
                const vec3 c1 = vec3(2.18, 0.24, 0.75);
                const vec3 c2 = vec3(-2.69, -7.46, 3.11);
                const vec3 c3 = vec3(6.13, 42.35, -28.52);
                const vec3 c4 = vec3(-11.11, -82.67, 60.14);
                const vec3 c5 = vec3(10.02, 71.41, -54.07);
                const vec3 c6 = vec3(-3.66, -22.93, 18.19);
                return c0+t*(c1+t*(c2+t*(c3+t*(c4+t*(c5+t*c6)))));
            }
            void vertex() {
                #define id2xy(id) ivec2((id)&0xff, (id)>>8)
                vec4 p = pointsTex(id2xy(ID.x));
                p.xyz -= center;
                varying vec3 color = color_map(p.w/(lastGen+1.));
                color = min(color, 0.9);
                color *= 1.0+max(0.0, 1.0-(lastGen-p.w)/10.0);
                VPos.xy = 1.9*(p.xy+XY*10.0)/(extent+30.);
            }`, FP: `vec4(color,1)*smoothstep(1.0, 0.8, length(XY))` }, target);

        return target;
    }

    frame(time) {
        requestAnimationFrame((t) => this.frame(t));
        
        if (time > 1000) {
            if (this.options.autonomous) {
                this.handleAutonomousBehavior();
            } else {
                if (this.graph.nodes.length <= this.options.growthLimit) {
                    this.graph.grow();
                    if (this.status) this.status.innerText = `${this.graph.nodes.length} nodes`;
                }
            }
        }
        
        if (this.simulation) this.simulation.tick(this.options.tickSteps);

        const DPR = devicePixelRatio;
        this.canvas.width = window.innerWidth * DPR;
        this.canvas.height = window.innerHeight * DPR;

        const framebuf = this.glsl({ Clear: 0 }, { tag: 'frame', msaa: 4 });
        this.renderGraph(framebuf);
        if (this.bloom) this.bloom.compose(framebuf);
        
        if (this.options.autonomous && this.transitionAlpha > 0) {
            this.glsl({ alpha: this.transitionAlpha, FP: `vec4(0, 0, 0, alpha)`, Blend: 's*sa+d*(1-sa)' });
        }
    }

    handleAutonomousBehavior() {
        if (this.transitioningOut) {
            this.transitionAlpha += 0.05;
            if (this.transitionAlpha >= 1.0) {
                this.pickNewRule();
                this.transitioningOut = false;
            }
        } else if (this.transitionAlpha > 0) {
            this.transitionAlpha -= 0.05;
        }
        
        if (!this.transitioningOut) {
            if (this.graph.nodes.length <= this.options.growthLimit) {
                this.graph.grow();
                if (this.status) this.status.innerText = `${this.graph.nodes.length} nodes (p=${this.graph.flipProb.toExponential(1)})`;
                this.autoTimer = 0;
                
                if (this.graph.nodes.length === this.prevNodeCount) {
                    this.stallTimer += 1;
                    if (this.stallTimer > 50 && this.stallTimer % 20 === 0) {
                        if (this.graph.flipProb === 0) this.graph.flipProb = 1e-4;
                        else this.graph.flipProb *= 10;
                        if (this.graph.flipProb > 0.5) this.graph.flipProb = 0.5;
                        this.params.flipProb = this.graph.flipProb;
                        this.gui.updateDisplay();
                    }
                    if (this.stallTimer > 200) {
                        this.transitioningOut = true;
                        this.consecutiveExplosions = 0;
                    }
                } else {
                    this.stallTimer = 0;
                }
                this.prevNodeCount = this.graph.nodes.length;
                
            } else {
                this.autoTimer += 1;
                let isExplosion = this.graph.lastGen < 30;
                if (isExplosion) {
                    this.consecutiveExplosions += 1;
                    if (this.consecutiveExplosions > 1) {
                        this.pickNewRule();
                    } else if (this.autoTimer > 10) {
                        this.transitioningOut = true;
                    }
                } else {
                    this.consecutiveExplosions = 0;
                    if (this.autoTimer > 200) {
                        this.transitioningOut = true;
                    }
                }
            }
        }
    }
}
