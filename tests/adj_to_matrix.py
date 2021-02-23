n, m = map(int, input().split())

adj = [[1000000]*n for i in range(n)]

for v in range(n):
    adj[v][v] = 0

for _ in range(m):
    u, v, c = map(int, input().split())
    adj[u][v] = c
penalties = list(map(int, input().split()))

for k in range(n):
    for i in range(n):
        for j in range(n):
            adj[i][j] = min(adj[i][j], adj[i][k] + adj[k][j])

print(n)
for row in adj:
    print(*row)
print(*penalties)
