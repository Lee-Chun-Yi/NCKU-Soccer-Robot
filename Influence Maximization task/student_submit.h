auto marginal_gain = [&](int u) -> double {

    if (seeds.count(u) || is_forbidden(u)) 
        return -1e18;

    size_t iu = id2idx[u];

    double score = 0.0;

    // (0) 基礎特性
    score -= GAMMA * pos_th[iu];
    score -= DELTA * in_strength[iu];
    score += ETA * neg_th_mag[iu];

    // =============================
    // (1) one-hop 正 / 負 覆蓋 (現有)
    // =============================
    for (int v : G.getNodeOutNeighbors(u)) {
        size_t j = id2idx[v];
        double w = G.getEdgeInfluence(u, v);

        if (w > 0) {
            double need = 1.0 - pos_coverage[j];
            if (need > 0) {
                double eff = min(need, w / pos_th[j]);
                score += LAMBDA * eff;
            }
        } else if (w < 0) {
            double needn = 1.0 - neg_coverage[j];
            if (needn > 0) {
                double effn = min(needn, fabs(w) / neg_th_mag[j]);
                score -= PHI * effn;
            }
        }
    }

    // =============================
    // (2) 2-hop positive influence
    // =============================
    const double decay = 0.55;
    for (int v : G.getNodeOutNeighbors(u)) {
        double w_uv = G.getEdgeInfluence(u, v);
        if (w_uv <= 0) continue;

        for (int x : G.getNodeOutNeighbors(v)) {
            double w_vx = G.getEdgeInfluence(v, x);
            if (w_vx <= 0) continue;

            size_t ix = id2idx[x];
            double add2 = (w_uv * w_vx) / pos_th[ix];
            double needx = 1.0 - pos_coverage[ix];
            if (needx > 0)
                score += decay * min(needx, add2);
        }
    }

    // =============================
    // (3) 2-hop negative risk
    // =============================
    const double decay_neg = 0.4;
    for (int v : G.getNodeOutNeighbors(u)) {
        double w_uv = G.getEdgeInfluence(u, v);
        if (w_uv >= 0) continue;

        for (int x : G.getNodeOutNeighbors(v)) {
            double w_vx = G.getEdgeInfluence(v, x);
            if (w_vx <= 0) continue;

            size_t ix = id2idx[x];
            double add2 = fabs(w_uv) * w_vx / neg_th_mag[ix];
            double need = 1.0 - neg_coverage[ix];
            if (need > 0)
                score -= decay_neg * min(need, add2);
        }
    }

    // =============================
    // (4) overlap penalty (避免集中)
    // =============================
    double overlapPenalty = 0.12;
    double overlap = 0.0;
    for (int v : G.getNodeOutNeighbors(u)) {
        size_t j = id2idx[v];
        overlap += pos_coverage[j]; 
    }
    score -= overlapPenalty * overlap;

    return score;
};
