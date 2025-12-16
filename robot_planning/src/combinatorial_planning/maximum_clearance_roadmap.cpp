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
        bool isTarget;      // È una vittima o un gate?
        bool isPath;        // È parte del sentiero di connessione protetta?
        Vertex exactPos;    
    };

    bool isValid(int x, int y, int width, int height) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    // --- Thinning Algorithm (Zhang-Suen) ---
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
                        // Protezione: Non rimuovere se non è Voronoi, se è Target o se è SENTIERO
                        if (!grid[x][y].isVoronoi || grid[x][y].isTarget || grid[x][y].isPath) continue;

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

    // --- Gradient Ascent: Connessione Protetta ---
    void connectTargetToSkeleton(int startX, int startY, int width, int height, std::vector<std::vector<PixelInfo>>& grid) {
        int cx = startX;
        int cy = startY;
        int maxSteps = width * height;
        int steps = 0;

        int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
        int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};

        while (steps < maxSteps) {
            // Se incontriamo un nodo Voronoi esistente (che non sia il target stesso o il nostro sentiero), siamo connessi
            if (grid[cx][cy].isVoronoi && !grid[cx][cy].isTarget && !grid[cx][cy].isPath) {
                break;
            }

            // MARCATURA FONDAMENTALE: Questo pixel fa parte del ponte e non deve essere cancellato
            grid[cx][cy].isVoronoi = true;
            grid[cx][cy].isPath = true; 

            double maxDist = grid[cx][cy].dist;
            int bestNx = -1, bestNy = -1;

            // Cerca il vicino con distanza maggiore (salita verso il centro)
            for (int i = 0; i < 8; ++i) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                if (isValid(nx, ny, width, height)) {
                    // Se tocchiamo un nodo Voronoi esistente (non target, non path attuale), connettiamoci subito
                    if (grid[nx][ny].isVoronoi && !grid[nx][ny].isTarget && !grid[nx][ny].isPath) {
                        bestNx = nx; bestNy = ny;
                        break; 
                    }
                    
                    // Altrimenti seguiamo il gradiente
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
                break; // Massimo locale raggiunto
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

        // Init: Aggiunto flag isPath (false di default)
        std::vector<std::vector<PixelInfo>> grid(width, std::vector<PixelInfo>(height, {std::numeric_limits<double>::max(), -1, false, -1, false, false, Vertex(0,0)}));
        std::queue<GridNode> q;

        // 1. Brushfire Init
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
                    currentInfo.isVoronoi = true;
                    neighborInfo.isVoronoi = true; 
                }
            }
        }

        // 3. Iniezione Target
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

        std::vector<GridNode> targetCells;
        for (const auto& t : targets) {
            int gx = std::round((t.x - minX) / gridResolution);
            int gy = std::round((t.y - minY) / gridResolution);

            if (isValid(gx, gy, width, height)) {
                grid[gx][gy].isTarget = true;
                grid[gx][gy].isVoronoi = true;
                grid[gx][gy].exactPos = t;
                targetCells.push_back({gx, gy});
            }
        }

        // Connessione Protetta
        for (const auto& cell : targetCells) {
            connectTargetToSkeleton(cell.x, cell.y, width, height, grid);
        }

        // 4. Thinning
        applyThinning(grid, width, height);

        // 5. Estrazione Nodi
        // MODIFICA: Il filtro originale sulla clearance causava nodi isolati.
        // Lo rimuoviamo per garantire che l'intero scheletro di Voronoi venga convertito in nodi.
        
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                if (grid[x][y].isVoronoi) {
                    // Manteniamo SEMPRE i nodi Voronoi per garantire la connettività.
                    // Se necessario, il path planner (A*) penalizzerà i nodi troppo vicini agli ostacoli
                    // (anche se MCR per definizione cerca già la massima distanza).
                    bool keep = true;

                    if (keep) {
                        Vertex v;
                        if (grid[x][y].isTarget) v = grid[x][y].exactPos;
                        else v = Vertex(minX + x * gridResolution, minY + y * gridResolution);
                        
                        grid[x][y].roadmapNodeIdx = roadmap->addVertex(v);
                    }
                }
            }
        }

        // 6. Connessione Archi
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                int uIdx = grid[x][y].roadmapNodeIdx;
                if (uIdx == -1) continue;

                for (int i = 0; i < 8; ++i) {
                    int nx = x + dx[i];
                    int ny = y + dy[i];

                    if (isValid(nx, ny, width, height)) {
                        int vIdx = grid[nx][ny].roadmapNodeIdx;
                        // Connetti solo se anche il vicino è un nodo valido (che ora è garantito se era Voronoi)
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