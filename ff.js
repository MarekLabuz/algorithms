const fs = require('fs')

class Graph {
  constructor () {
    this.vertexes = {}
    this.discovered = new Set()
    this.flows = {}
  }

  addEdge (v1, v2, capacity) {
    this.vertexes = {
      ...this.vertexes,
      [v1]: {
        ...this.vertexes[v1],
        [v2]: { capacity, flow: 0 }
      }
    }
  }

  reset () {
    this.vertexes = Object.keys(this.vertexes).reduce((acc, id) => ({
      ...acc,
      [id]: Object.keys(this.vertexes[id]).reduce((acc2, id2) => ({
        ...acc2,
        [id2]: Object.keys(this.vertexes[id][id2]).reduce((acc3, id3) => ({
          ...acc3,
          flow: 0
        }), this.vertexes[id][id2])
      }), this.vertexes[id])
    }), this.vertexes)
    this.discovered.clear()
  }
}

const graph = new Graph()

const DFS = (a, b, path = [a]) => {
  graph.discovered.add(a)
  if (a === b) {
    return path
  } else {
    const connectedVertexes = Object.keys(graph.vertexes[a])
    for (const vertex of connectedVertexes) {
      const { capacity, flow } = graph.vertexes[a][vertex]
      const cf = capacity - flow
      if (!graph.discovered.has(vertex) && cf > 0) {
        const p = DFS(vertex, b, [...path, vertex])
        if (p) {
          return p
        }
      }
    }
  }
  return null
}

const constructPath = (s, meta) => {
  const result = [s]

  let state = s
  while (true) {
    const row = meta[state]
    if (row) {
      state = row[0]
      result.push(row[1])
    } else {
      break
    }
  }

  return result.reverse().slice(1)
}

const BFS = (a, b) => {
  const queue = []
  graph.discovered.clear()
  const meta = {}

  meta[a] = [null, null]
  queue.push(a)

  while (queue.length) {
    const u = queue.shift()

    if (u === b) {
      return constructPath(u, meta)
    }

    const connectedVertexes = Object.keys(graph.vertexes[u])
    for (const v of connectedVertexes) {
      if (graph.discovered.has(v)) {
        continue
      }

      const { capacity, flow } = graph.vertexes[u][v]
      const cf = capacity - flow

      if (!queue.includes(v) && cf > 0) {
        meta[v] = [u, u]
        queue.push(v)
      }
    }

    graph.discovered.add(u)
  }
}

fs.readFileSync('graf2.txt', 'utf-8')
  .split('\n')
  .filter(v => v)
  .map(v => v.split(';').map(e => parseFloat(e)))
  .forEach(([v1, v2, capacity]) => {
    graph.addEdge(v1, v2, capacity)
  })

const ff = (a, b, algorithm) => {
  let path = algorithm(a, b)
  while (path) {
    const minCf = path.reduce((minCf, curr, i) => {
      if (!path[i - 1]) {
        return minCf
      }
      const { capacity, flow } = graph.vertexes[path[i - 1]][curr]
      const cf = capacity - flow
      return cf < minCf ? cf : minCf
    }, Infinity)

    for (let i = 1; i < path.length; i += 1) {
      const edge = graph.vertexes[path[i - 1]][path[i]]
      edge.flow += minCf
    }

    graph.discovered.clear()
    path = algorithm(a, b)
  }
}

const sumFlow = id => Object.values(graph.vertexes[id])
  .reduce((acc, { flow }) => acc + flow, 0)

ff('1', '7', DFS)
console.log('DFS', sumFlow('1'))

graph.reset()
ff('1', '7', BFS)
console.log('BFS', sumFlow('1'))
