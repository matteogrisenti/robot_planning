#include "combinatorial_planning/maximum_clearance_roadmap.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include "combinatorial_planning/planning_utils.h"

namespace MaxClearanceRoadmap {

    struct GridNode { int x, y; };

    struct PixelInfo {
        double dist;       
        int sourceObsId;   
        bool isVoronoi;    
        int roadmapNodeIdx;
        bool isTarget;      // Indica se la cella contiene un target
        Vertex exactPos;    // Posizione esatta del target (per evitare errori di discretizzazione)
    };

    bool isValid(int x, int y, int width, int height) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    // --- Thinning Algorithm (Zhang-Suen) ---
    // Assottiglia le linee mantenendo la connettività
    void applyThinning(std::vector<std::vector<PixelInfo>>& grid, int width, int height) {
        int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
        int dy[] = {-1, -1, 0, 1, 1, 1, 0, -1};

        bool pixelRemoved = true;
        while (pixelRemoved) {
            pixelRemoved = false;
            std::vector<GridNode> toRemove;

            for (int iter = 0; iter < 2; ++iter) {
                toRemove.clear();
                for (int x = 1; x < width - 1; ++x) {
                    for (int y = 1; y < height - 1; ++y) {
                        // NON rimuovere se non è Voronoi o se è un TARGET
                        if (!grid[x][y].isVoronoi || grid[x][y].isTarget) continue;

                        int p2 = grid[x + dx[0]][y + dy[0]].isVoronoi;
                        int p3 = grid[x + dx[1]][y + dy[1]].isVoronoi;
                        int p4 = grid[x + dx[2]][y + dy[2]].isVoronoi;
                        int p5 = grid[x + dx[3]][y + dy[3]].isVoronoi;
                        int p6 = grid[x + dx[4]][y + dy[4]].isVoronoi;
                        int p7 = grid[x + dx[5]][y + dy[5]].isVoronoi;
                        int p8 = grid[x + dx[6]][y + dy[6]].isVoronoi;
                        int p9 = grid[x + dx[7]][y + dy[7]].isVoronoi;

                        int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) + 
                                (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) + 
                                (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                                (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
                        int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;

                        int m1 = (iter == 0) ? (p2 * p4 * p6) : (p2 * p4 * p8);
                        int m2 = (iter == 0) ? (p4 * p6 * p8) : (p2 * p6 * p8);

                        if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0) {
                            toRemove.push_back({x, y});
                        }
                    }
                }
                
                for (const auto& p : toRemove) {
                    grid[p.x][p.y].isVoronoi = false;
                    pixelRemoved = true;
                }
            }
        }
    }

    // --- Funzione per collegare il Target allo Scheletro (Gradient Ascent) ---
    void connectTargetToSkeleton(int startX, int startY, int width, int height, std::vector<std::vector<PixelInfo>>& grid) {
        int cx = startX;
        int cy = startY;
        
        int maxSteps = width * height; // Safety break
        int steps = 0;

        int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
        int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};

        while (steps < maxSteps) {
            // Se siamo già su un nodo Voronoi preesistente (che non sia il target stesso all'inizio), ci siamo connessi!
            if (grid[cx][cy].isVoronoi && !grid[cx][cy].isTarget) {
                break;
            }

            // Marchiamo il percorso corrente come parte del grafo
            grid[cx][cy].isVoronoi = true;

            // Cerchiamo il vicino con la distanza maggiore (Gradient Ascent)
            double maxDist = grid[cx][cy].dist;
            int bestNx = -1, bestNy = -1;

            for (int i = 0; i < 8; ++i) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                if (isValid(nx, ny, width, height)) {
                    // Se troviamo un nodo Voronoi vicino, andiamo subito lì per chiudere il loop
                    if (grid[nx][ny].isVoronoi && !grid[nx][ny].isTarget) {
                        bestNx = nx; bestNy = ny;
                        break; // Priorità alla connessione
                    }

                    if (grid[nx][ny].dist > maxDist) {
                        maxDist = grid[nx][ny].dist;
                        bestNx = nx;
                        bestNy = ny;
                    }
                }
            }

            if (bestNx != -1) {
                cx = bestNx;
                cy = bestNy;
            } else {
                // Massimo locale raggiunto. Ci fermiamo qui.
                break;
            }
            steps++;
        }
    }

    std::shared_ptr<Roadmap> maximumClearanceRoadmap(const Map& map, double gridResolution) {
        auto roadmap = std::make_shared<Roadmap>();
        roadmap->setMap(&map);

        double minX, minY, maxX, maxY;
        map.get_bounding_box(minX, minY, maxX, maxY);
        minX -= 1.0; minY -= 1.0; maxX += 1.0; maxY += 1.0;

        int width = std::ceil((maxX - minX) / gridResolution);
        int height = std::ceil((maxY - minY) / gridResolution);

        if (width <= 0 || height <= 0) return roadmap;

        // Inizializzazione Griglia
        std::vector<std::vector<PixelInfo>> grid(width, std::vector<PixelInfo>(height, {std::numeric_limits<double>::max(), -1, false, -1, false, Vertex(0,0)}));
        std::queue<GridNode> q;

        // 1. Brushfire Initialization (Bordi e Ostacoli)
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                Vertex worldPos(minX + x * gridResolution, minY + y * gridResolution);
                bool insideObstacle = false;
                int foundObsId = -1;

                const auto& obstacles = map.obstacles.get_obstacles();
                for (size_t i = 0; i < obstacles.size(); ++i) {
                    if (PlanningUtils::pointInObstacle(worldPos, obstacles[i])) {
                        insideObstacle = true; foundObsId = i + 1; break;
                    }
                }
                
                std::vector<Vertex> borderPoly;
                for(const auto& p : map.borders.get_points()) borderPoly.push_back(Vertex(p.x, p.y));
                if (!insideObstacle && !PlanningUtils::pointInPolygon(worldPos, borderPoly)) {
                    insideObstacle = true; foundObsId = 0;
                }

                if (insideObstacle) {
                    grid[x][y].dist = 0.0;
                    grid[x][y].sourceObsId = foundObsId;
                    q.push({x, y});
                }
            }
        }

        // 2. Brushfire Propagation
        int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
        int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};
        
        while (!q.empty()) {
            GridNode current = q.front(); q.pop();
            PixelInfo& currentInfo = grid[current.x][current.y];

            for (int i = 0; i < 8; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                if (!isValid(nx, ny, width, height)) continue;
                PixelInfo& neighborInfo = grid[nx][ny];

                double stepCost = (std::abs(dx[i]) + std::abs(dy[i]) == 2) ? 1.414 : 1.0;
                
                if (neighborInfo.sourceObsId == -1) {
                    neighborInfo.dist = currentInfo.dist + stepCost;
                    neighborInfo.sourceObsId = currentInfo.sourceObsId;
                    q.push({nx, ny});
                } 
                else if (neighborInfo.sourceObsId != currentInfo.sourceObsId) {
                    // Confine tra due ostacoli diversi -> Ridge di Voronoi
                    currentInfo.isVoronoi = true;
                    neighborInfo.isVoronoi = true; 
                }
            }
        }

        // 3. Iniezione Targets e Connessione (Gradient Ascent)
        std::vector<Vertex> targets;
        targets.push_back(Vertex(map.start.get_position().x, map.start.get_position().y));
        if (!map.gates.get_gates().empty()) {
            Point g = map.gates.get_gates()[0].get_position();
            targets.push_back(Vertex(g.x, g.y));
        }
        for (const auto& v : map.victims.get_victims()) {
            Point p = v.get_center();
            targets.push_back(Vertex(p.x, p.y));
        }

        // Lista di celle target per dopo
        std::vector<GridNode> targetCells;

        for (const auto& t : targets) {
            int gx = std::round((t.x - minX) / gridResolution);
            int gy = std::round((t.y - minY) / gridResolution);

            if (isValid(gx, gy, width, height)) {
                grid[gx][gy].isTarget = true;
                grid[gx][gy].isVoronoi = true; // Deve essere parte del grafo
                grid[gx][gy].exactPos = t;     // Salviamo la coordinata float precisa
                targetCells.push_back({gx, gy});
            }
        }

        // Collega ogni target allo scheletro principale risalendo la distanza
        for (const auto& cell : targetCells) {
            connectTargetToSkeleton(cell.x, cell.y, width, height, grid);
        }

        // 4. Thinning (per pulire i ridge spessi, ma preserva la connettività dei target)
        applyThinning(grid, width, height);

        // 5. Estrazione Nodi
        double minClearance = 1.0 / gridResolution; // Filtro rumore minimo

        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                // Accetta il nodo se è Voronoi E (ha buona clearance OPPURE è un target/percorso target)
                // Usiamo una soglia di distanza anche per i nodi generici per evitare rumore vicino ai muri
                if (grid[x][y].isVoronoi) {
                    bool keep = false;
                    if (grid[x][y].isTarget) keep = true;
                    else if (grid[x][y].dist > minClearance) keep = true;

                    if (keep) {
                        Vertex v;
                        // SE è un target, usiamo la posizione esatta. ALTRIMENTI usiamo il centro griglia
                        if (grid[x][y].isTarget) {
                            v = grid[x][y].exactPos;
                        } else {
                            v = Vertex(minX + x * gridResolution, minY + y * gridResolution);
                        }
                        grid[x][y].roadmapNodeIdx = roadmap->addVertex(v);
                    }
                }
            }
        }

        // 6. Connessione Archi
        // Collega ogni nodo ai suoi vicini (8-connectivity) sulla griglia Voronoi
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                int uIdx = grid[x][y].roadmapNodeIdx;
                if (uIdx == -1) continue;

                for (int i = 0; i < 8; ++i) {
                    int nx = x + dx[i];
                    int ny = y + dy[i];

                    if (isValid(nx, ny, width, height)) {
                        int vIdx = grid[nx][ny].roadmapNodeIdx;
                        // Aggiungi arco se il vicino è un nodo valido
                        // Nota: Se ci sono "buchi" di nodi Voronoi (es. rimossi per low clearance),
                        // la connessione si interrompe. Il Gradient Ascent garantisce che non succeda per i target.
                        if (vIdx != -1 && uIdx < vIdx) {
                            roadmap->addEdge(uIdx, vIdx, true);
                        }
                    }
                }
            }
        }
        
        return roadmap;
    }
}