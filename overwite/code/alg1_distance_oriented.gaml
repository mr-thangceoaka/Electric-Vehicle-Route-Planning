model alg1_solver_distance_oriented_evrp

global {
    int n_customers <- 20;
    string distribution <- "U";
    int data_seed <- 1234;

    int DEPOT_ID <- 0;
    float SERVICE_MIN <- 5.0;
    float START_TIME_MIN <- 7.0 * 60.0;
    float BATTERY_CAPACITY_KWH <- 30.0;
    float RESERVE_KWH <- 3.0;
    float ENERGY_RATE_KWH_PER_KM <- 0.2;
    float CHARGER_POWER_KW <- 50.0;
    float CHARGE_RATE_KWH_PER_MIN <- CHARGER_POWER_KW / 60.0;
    float CONGESTED_EDGE_DENSITY <- 0.80;
    float INFEASIBLE <- 1000000000000.0;
    float EPS <- 0.000000001;

    list<ev_node> all_nodes <- [];
    list<ev_node> customers <- [];
    list<ev_node> stations <- [];
    list<ev_edge> edges <- [];

    init {
        do generate_instance;
        create ev_vehicle number: 1 {
            name <- "Alg1 vehicle";
            location <- node_by_id(DEPOT_ID).location;
        }
    }

    ev_node node_by_id(int id) {
        loop n over: all_nodes {
            if n.node_id = id { return n; }
        }
        return nil;
    }

    float euclidean(ev_node a, ev_node b) {
        return sqrt((a.location.x - b.location.x) ^ 2 + (a.location.y - b.location.y) ^ 2);
    }

    float clamp(float v, float lo, float hi) {
        return max(lo, min(hi, v));
    }

    list<point> station_coordinates(int m) {
        list<point> base <- [
            {35,50}, {65,50}, {50,35}, {50,65},
            {25,25}, {75,25}, {25,75}, {75,75},
            {50,15}, {50,85}, {15,50}, {85,50}
        ];
        if m <= length(base) {
            return base[0::m - 1];
        }
        return base;
    }

    void generate_instance {
        create ev_node number: 1 {
            node_id <- DEPOT_ID;
            kind <- "depot";
            location <- {50, 50};
            wait_base_min <- 0.0;
        }

        if distribution = "U" {
            loop cid from: 1 to: n_customers {
                create ev_node number: 1 {
                    node_id <- cid;
                    kind <- "customer";
                    location <- {rnd(5.0, 95.0), rnd(5.0, 95.0)};
                    wait_base_min <- 0.0;
                }
            }
        } else {
            list<point> centers <- [{25,25}, {75,25}, {30,75}, {75,75}, {50,50}];
            loop cid from: 1 to: n_customers {
                point c <- one_of(centers);
                create ev_node number: 1 {
                    node_id <- cid;
                    kind <- "customer";
                    location <- {clamp(c.x + rnd(-8.0, 8.0), 2.0, 98.0), clamp(c.y + rnd(-8.0, 8.0), 2.0, 98.0)};
                    wait_base_min <- 0.0;
                }
            }
        }

        int m_stations <- max(1, round(0.10 * n_customers));
        list<point> st_coords <- station_coordinates(m_stations);
        loop k from: 0 to: m_stations - 1 {
            create ev_node number: 1 {
                node_id <- n_customers + 1 + k;
                kind <- "station";
                location <- st_coords[k];
                wait_base_min <- rnd(10.0, 35.0);
            }
        }

        all_nodes <- ev_node as list;
        customers <- ev_node where (each.kind = "customer") as list;
        stations <- ev_node where (each.kind = "station") as list;

        loop i from: 0 to: length(all_nodes) - 1 {
            loop j from: i + 1 to: length(all_nodes) - 1 {
                ev_node a <- all_nodes[i];
                ev_node b <- all_nodes[j];
                create ev_edge number: 1 {
                    from_node <- a;
                    to_node <- b;
                    distance_km <- euclidean(a, b);
                    congested <- flip(CONGESTED_EDGE_DENSITY);
                }
            }
        }
        edges <- ev_edge as list;
    }

    ev_edge edge_between(ev_node a, ev_node b) {
        loop e over: edges {
            if (e.from_node = a and e.to_node = b) or (e.from_node = b and e.to_node = a) {
                return e;
            }
        }
        return nil;
    }

    float distance_between(ev_node a, ev_node b) {
        if a = b { return 0.0; }
        ev_edge e <- edge_between(a, b);
        if e = nil { return euclidean(a, b); }
        return e.distance_km;
    }

    bool is_congested(ev_node a, ev_node b) {
        ev_edge e <- edge_between(a, b);
        return e != nil and e.congested;
    }

    float energy_between(ev_node a, ev_node b) {
        return ENERGY_RATE_KWH_PER_KM * distance_between(a, b);
    }

    float travel_speed_kmph(ev_node a, ev_node b, float depart_min) {
        if not is_congested(a, b) { return 60.0; }
        float m <- depart_min mod 1440.0;
        if (m >= 420 and m < 540) or (m >= 990 and m < 1140) { return 20.0; }
        if (m >= 330 and m < 420) or (m >= 540 and m < 990) or (m >= 1140 and m < 1320) { return 40.0; }
        return 60.0;
    }

    float travel_time_min(ev_node a, ev_node b, float depart_min) {
        return distance_between(a, b) / travel_speed_kmph(a, b, depart_min) * 60.0;
    }

    float station_wait_min(ev_node s, float arrival_min) {
        if s.kind != "station" { return 0.0; }
        float m <- arrival_min mod 1440.0;
        if not (m >= 900 and m < 1290) { return 0.0; }
        float center <- 18.0 * 60.0;
        float half_width <- 195.0;
        float shape <- max(0.0, 1.0 - abs(m - center) / half_width);
        return s.wait_base_min * (0.40 + 0.60 * shape);
    }
}

species ev_node {
    int node_id;
    string kind;
    float wait_base_min <- 0.0;

    aspect base {
        if kind = "depot" { draw circle(2.0) color: #black; }
        else if kind = "station" { draw square(2.0) color: #green; }
        else { draw circle(1.5) color: #blue; }
    }
}

species ev_edge {
    ev_node from_node;
    ev_node to_node;
    float distance_km;
    bool congested;
    geometry shape <- line([from_node.location, to_node.location]);

    aspect base {
        draw shape color: (congested ? #red : #gray) width: 0.4;
    }
}

species ev_vehicle {
    string name;
    ev_node current_node <- node_by_id(DEPOT_ID);
    float now_min <- START_TIME_MIN;
    float battery_kwh <- BATTERY_CAPACITY_KWH;
    float total_time_min <- 0.0;
    float distance_km <- 0.0;
    float drive_min <- 0.0;
    float wait_min <- 0.0;
    float charge_min <- 0.0;
    float service_min <- 0.0;
    int charges <- 0;
    bool feasible <- true;
    bool finished <- false;
    string reason <- "";
    list<ev_node> plan <- [];
    list<ev_node> remaining <- [];
    list<ev_node> executed_route <- [];

    init {
        plan <- solver_customer_plan;
        remaining <- copy(plan);
        executed_route <- [current_node];
    }

    reflex run_alg1 when: not finished {
        do execute_distance_policy;
        finished <- true;
        write "Alg1 solver-plan feasible=" + feasible + ", T=" + total_time_min + " min, D=" + distance_km + " km, charges=" + charges;
    }

    list<ev_node> nearest_neighbor_customer_plan {
        list<ev_node> unvisited <- copy(customers);
        list<ev_node> out <- [];
        ev_node cur <- node_by_id(DEPOT_ID);
        while length(unvisited) > 0 {
            ev_node nxt <- unvisited min_of distance_between(cur, each);
            out <- out + nxt;
            unvisited <- unvisited - nxt;
            cur <- nxt;
        }
        return out;
    }



    float plan_distance(list<ev_node> p) {
        if length(p) = 0 { return 0.0; }
        float total <- 0.0;
        ev_node depot <- node_by_id(DEPOT_ID);
        ev_node cur <- depot;
        loop n over: p {
            total <- total + distance_between(cur, n);
            cur <- n;
        }
        total <- total + distance_between(cur, depot);
        return total;
    }

    list<ev_node> two_opt_swap(list<ev_node> p, int i, int j) {
        list<ev_node> out <- [];
        loop k from: 0 to: i - 1 {
            out <- out + p[k];
        }
        loop k from: j to: i step: -1 {
            out <- out + p[k];
        }
        loop k from: j + 1 to: length(p) - 1 {
            out <- out + p[k];
        }
        return out;
    }

    list<ev_node> solver_customer_plan {
        list<ev_node> p <- nearest_neighbor_customer_plan;
        bool improved <- true;
        int pass <- 0;
        while improved and pass < 30 {
            improved <- false;
            pass <- pass + 1;
            float best_d <- plan_distance(p);
            loop i from: 0 to: length(p) - 2 {
                loop j from: i + 1 to: length(p) - 1 {
                    list<ev_node> cand <- two_opt_swap(p, i, j);
                    float cand_d <- plan_distance(cand);
                    if cand_d + EPS < best_d {
                        p <- cand;
                        best_d <- cand_d;
                        improved <- true;
                    }
                }
            }
        }
        return p;
    }

    bool can_reach(ev_node a, ev_node b, float bat) {
        return bat + EPS >= energy_between(a, b) + RESERVE_KWH;
    }

    bool safe_customer_move(ev_node a, ev_node b, float bat) {
        if not can_reach(a, b, bat) { return false; }
        float b_after <- bat - energy_between(a, b);
        list<ev_node> targets <- stations + node_by_id(DEPOT_ID);
        loop t over: targets {
            if b_after + EPS >= energy_between(b, t) + RESERVE_KWH { return true; }
        }
        return false;
    }

    list<ev_node> reachable_stations_from(ev_node a, float bat) {
        list<ev_node> out <- [];
        loop s over: stations {
            if s != a and can_reach(a, s, bat) { out <- out + s; }
        }
        return out;
    }

    ev_node choose_nearest_reachable_station(ev_node a, float bat) {
        list<ev_node> candidates <- reachable_stations_from(a, bat);
        if length(candidates) = 0 { return nil; }
        return candidates min_of distance_between(a, each);
    }

    bool move_to_node(ev_node target) {
        if target = current_node {
            feasible <- false;
            reason <- "attempted self move";
            return false;
        }
        if not can_reach(current_node, target, battery_kwh) {
            feasible <- false;
            reason <- "battery infeasible";
            return false;
        }

        float d <- distance_between(current_node, target);
        float travel <- travel_time_min(current_node, target, now_min);
        float energy <- energy_between(current_node, target);
        now_min <- now_min + travel;
        battery_kwh <- battery_kwh - energy;
        distance_km <- distance_km + d;
        drive_min <- drive_min + travel;
        current_node <- target;
        location <- target.location;
        executed_route <- executed_route + target;

        if target.kind = "customer" {
            now_min <- now_min + SERVICE_MIN;
            service_min <- service_min + SERVICE_MIN;
        } else if target.kind = "station" {
            float wait <- station_wait_min(target, now_min);
            float charge <- max(0.0, BATTERY_CAPACITY_KWH - battery_kwh) / CHARGE_RATE_KWH_PER_MIN;
            now_min <- now_min + wait + charge;
            battery_kwh <- BATTERY_CAPACITY_KWH;
            wait_min <- wait_min + wait;
            charge_min <- charge_min + charge;
            charges <- charges + 1;
        }
        return true;
    }

    void execute_distance_policy {
        int steps <- 0;
        while length(remaining) > 0 and feasible {
            steps <- steps + 1;
            if steps > 10000 {
                feasible <- false;
                reason <- "max steps exceeded";
                break;
            }
            ev_node target <- remaining[0];
            if safe_customer_move(current_node, target, battery_kwh) {
                if do move_to_node(target) {
                    remaining <- remaining - target;
                }
            } else {
                ev_node station <- choose_nearest_reachable_station(current_node, battery_kwh);
                if station = nil {
                    feasible <- false;
                    reason <- "no reachable station";
                    break;
                }
                do move_to_node(station);
            }
        }

        ev_node depot <- node_by_id(DEPOT_ID);
        while feasible and current_node != depot {
            if can_reach(current_node, depot, battery_kwh) {
                do move_to_node(depot);
            } else {
                ev_node station <- choose_nearest_reachable_station(current_node, battery_kwh);
                if station = nil {
                    feasible <- false;
                    reason <- "cannot return to depot";
                    break;
                }
                do move_to_node(station);
            }
        }
        total_time_min <- feasible ? now_min - START_TIME_MIN : INFEASIBLE;
    }

    aspect base {
        draw circle(2.5) color: #orange;
    }
}

experiment alg1_solver_gui type: gui {
    parameter "Number of customers" var: n_customers min: 5 max: 80 step: 5;
    parameter "Distribution" var: distribution among: ["U", "M"];

    output {
        display "EVRP Alg1 Solver Plan" {
            species ev_edge aspect: base;
            species ev_node aspect: base;
            species ev_vehicle aspect: base;
        }
        monitor "Total time (min)" value: first(ev_vehicle).total_time_min;
        monitor "Distance (km)" value: first(ev_vehicle).distance_km;
        monitor "Solver plan distance" value: first(ev_vehicle).plan_distance(first(ev_vehicle).plan);
        monitor "Charges" value: first(ev_vehicle).charges;
        monitor "Feasible" value: first(ev_vehicle).feasible;
    }
}
