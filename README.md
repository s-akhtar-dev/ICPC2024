# ICPC2024

## Data Structure

**Arrays and Vectors**
- Use `vector<int> v` for dynamic arrays
- Access elements with `v[i]`
- Add elements with `v.push_back(x)`
- Get size with `v.size()`

**Stack and Queue**
- Stack: `stack<int> s`
  - Push: `s.push(x)`
  - Pop: `s.pop()`
  - Top: `s.top()`
- Queue: `queue<int> q`
  - Push: `q.push(x)`
  - Pop: `q.pop()`
  - Front: `q.front()`

**Priority Queue**
```cpp
priority_queue<int> pq; // Max heap
priority_queue<int, vector<int>, greater<int>> pq; // Min heap
```

**Set and Map**
- Set: `set<int> s`
- Map: `map<int, int> m`

## Algorithms

**Sorting**
```cpp
sort(v.begin(), v.end());
```

**Binary Search**
```cpp
int binarySearch(vector<int>& arr, int target) {
    int left = 0, right = arr.size() - 1;
    while (left <= right) {
        int mid = left + (right - left) / 2;
        if (arr[mid] == target) return mid;
        if (arr[mid] < target) left = mid + 1;
        else right = mid - 1;
    }
    return -1;
}
```

**Depth-First Search (DFS)**
```cpp
void dfs(vector<vector<int>>& graph, int node, vector<bool>& visited) {
    visited[node] = true;
    for (int neighbor : graph[node]) {
        if (!visited[neighbor]) {
            dfs(graph, neighbor, visited);
        }
    }
}
```

**Breadth-First Search (BFS)**
```cpp
void bfs(vector<vector<int>>& graph, int start) {
    queue<int> q;
    vector<bool> visited(graph.size(), false);
    q.push(start);
    visited[start] = true;
    while (!q.empty()) {
        int node = q.front();
        q.pop();
        for (int neighbor : graph[node]) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                q.push(neighbor);
            }
        }
    }
}
```

## Dynamic Programming

**Fibonacci Sequence**
```cpp
vector<int> fib(int n) {
    vector<int> dp(n + 1);
    dp[0] = 0, dp[1] = 1;
    for (int i = 2; i <= n; i++) {
        dp[i] = dp[i-1] + dp[i-2];
    }
    return dp;
}
```

**Longest Common Subsequence**
```cpp
int lcs(string s1, string s2) {
    int m = s1.length(), n = s2.length();
    vector<vector<int>> dp(m + 1, vector<int>(n + 1, 0));
    for (int i = 1; i <= m; i++) {
        for (int j = 1; j <= n; j++) {
            if (s1[i-1] == s2[j-1]) dp[i][j] = dp[i-1][j-1] + 1;
            else dp[i][j] = max(dp[i-1][j], dp[i][j-1]);
        }
    }
    return dp[m][n];
}
```

## Graph Algorithms

**Dijkstra's Shortest Path**
```cpp
vector<int> dijkstra(vector<vector<pair<int, int>>>& graph, int start) {
    int n = graph.size();
    vector<int> dist(n, INT_MAX);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    dist[start] = 0;
    pq.push({0, start});
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        for (auto& [v, weight] : graph[u]) {
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}
```

**Union-Find (Disjoint Set)**
```cpp
class UnionFind {
    vector<int> parent, rank;
public:
    UnionFind(int n) : parent(n), rank(n, 0) {
        iota(parent.begin(), parent.end(), 0);
    }
    int find(int x) {
        if (parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    }
    void unite(int x, int y) {
        int px = find(x), py = find(y);
        if (px == py) return;
        if (rank[px] < rank[py]) parent[px] = py;
        else if (rank[px] > rank[py]) parent[py] = px;
        else { parent[py] = px; rank[px]++; }
    }
};
```

## Math

**Greatest Common Divisor (GCD)**
```cpp
int gcd(int a, int b) {
    return b == 0 ? a : gcd(b, a % b);
}
```

**Fast Exponentiation**
```cpp
long long fastPow(long long base, long long exp, long long mod) {
    long long result = 1;
    while (exp > 0) {
        if (exp & 1) result = (result * base) % mod;
        base = (base * base) % mod;
        exp >>= 1;
    }
    return result;
}
```

## String Algorithms

**KMP (Knuth-Morris-Pratt)**
```cpp
vector<int> computeLPS(string pattern) {
    int m = pattern.length();
    vector<int> lps(m, 0);
    int len = 0, i = 1;
    while (i < m) {
        if (pattern[i] == pattern[len]) {
            len++;
            lps[i] = len;
            i++;
        } else {
            if (len != 0) len = lps[len - 1];
            else {
                lps[i] = 0;
                i++;
            }
        }
    }
    return lps;
}
```

## Cheatsheet

- Time complexity: O(1) < O(log n) < O(n) < O(n log n) < O(n^2) < O(2^n) < O(n!)
- Modular arithmetic: (a + b) % m = ((a % m) + (b % m)) % m
- Bitwise operations: AND (&), OR (|), XOR (^), NOT (~), Left shift (<<), Right shift (>>)
- Prime check: Trial division up to sqrt(n)
- Sieve of Eratosthenes for prime generation
- Binary representation: n & (n-1) removes the last set bit
- Useful STL: lower_bound(), upper_bound(), next_permutation(), prev_permutation()
