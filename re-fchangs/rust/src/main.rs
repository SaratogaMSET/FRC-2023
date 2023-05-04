use std::{
    io::{self, BufRead, Write},
    net::TcpStream,
};

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use rust::amcl::Amcl;

#[derive(Serialize, Deserialize)]
struct EstimateData {
    id: i64,
    x: f64,
    y: f64,
    w: f64,
}

fn main() {
    // Create a TCP stream that connects to the server on port 8080
    let stream = TcpStream::connect("127.0.0.1:8080").unwrap();
    println!("Connected to server");

    // Get the input and output of the stream
    let mut reader = io::BufReader::new(&stream);
    let mut writer = io::BufWriter::new(&stream);

    // Initialize the particle filter
    let mut amcl: Amcl = Amcl::default();
    amcl.init();
    println!("Initialized particle filter");

    loop {
        // Read a line from the server
        let mut line = String::new();
        reader.read_line(&mut line).unwrap();
        let json_in: Value = serde_json::from_str(line.trim()).unwrap();

        let odom_id = json_in["OdomID"].as_i64().unwrap();
        let odom_array = json_in["OdomDeltas"].as_array().unwrap();
        let vision_id = json_in["VisionID"].as_i64().unwrap();
        let has_targets = json_in["HasTargets"].as_bool().unwrap();
        let tag_id = json_in["TagID"].as_i64().unwrap();
        let tag_distances = json_in["TagDistances"].as_array().unwrap();
        let campose = json_in["Campose"].as_array().unwrap();

        // Filter the particles
        amcl.update_odometry(
            odom_array.get(0).unwrap().as_f64().unwrap(),
            odom_array.get(1).unwrap().as_f64().unwrap(),
            odom_array.get(2).unwrap().as_f64().unwrap(),
        );
        if odom_id == vision_id && has_targets {
            amcl.tag_scanning(
                tag_id as i32,
                [
                    tag_distances.get(0).unwrap().as_f64().unwrap(),
                    tag_distances.get(1).unwrap().as_f64().unwrap(),
                    tag_distances.get(2).unwrap().as_f64().unwrap(),
                    tag_distances.get(3).unwrap().as_f64().unwrap(),
                    tag_distances.get(4).unwrap().as_f64().unwrap(),
                    tag_distances.get(5).unwrap().as_f64().unwrap(),
                    tag_distances.get(6).unwrap().as_f64().unwrap(),
                    tag_distances.get(7).unwrap().as_f64().unwrap(),
                ],
                [
                    campose.get(0).unwrap().as_f64().unwrap(),
                    campose.get(1).unwrap().as_f64().unwrap(),
                    campose.get(2).unwrap().as_f64().unwrap(),
                ],
            );
        }

        amcl.compute_weighted_average(); // potential FIXME: we might want to get rid of this line
        let out_data = EstimateData {
            id: odom_id,
            x: amcl.get_weighted_average().x(),
            y: amcl.get_weighted_average().y(),
            w: amcl.get_weighted_average().w(),
        };
        let json_out = json!(out_data);

        // Write a line to the server
        writeln!(writer, "{}", json_out).unwrap();
        writer.flush().unwrap();
    }
}
