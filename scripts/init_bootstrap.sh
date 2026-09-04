#!/bin/sh
set -euxo pipefail

# 1) Chown volumes once using sentinel files
for d in \
  /ros2_build \
  /ros2_install \
  /ros2_log \
  /platformio_cache \
  /mcu_build_artifacts \
  /mcu_build_artifacts_microros \
  /mcu_lib_external \
; do
  echo "[bootstrap] Ensuring directory exists: $d"
  mkdir -p "$d"
  if [ -f "$d/.racer_owner_set" ]; then
    echo "[bootstrap] Ownership already set for $d; skipping"
  else
    echo "[bootstrap] Chowning recursively to 1000:1000: $d"
    chown -R 1000:1000 "$d"
    echo "owner=1000:1000" > "$d/.racer_owner_set"
  fi
done

# 2) Seed libs_external once using sentinel file
DATA=/mcu_lib_external
MOUNT_SEED=/seed
BAKED_SEED=/seed_baked
SEED_SRC=""

echo "[bootstrap] Preparing libs_external seed"
mkdir -p "$DATA"

if [ -f "$DATA/.racer_libs_seeded" ]; then
  echo "[bootstrap] Sentinel found: $DATA/.racer_libs_seeded — skipping seed"
else
  echo "seeded_from=$SEED_SRC" > "$DATA/.racer_libs_seeded"
  exit 0
fi

if [ -d "$MOUNT_SEED" ] && [ -n "$(ls -A "$MOUNT_SEED" 2>/dev/null)" ]; then
  SEED_SRC="$MOUNT_SEED"
  echo "[bootstrap] Using mounted seed: $SEED_SRC"
elif [ -d "$BAKED_SEED" ] && [ -n "$(ls -A "$BAKED_SEED" 2>/dev/null)" ]; then
  SEED_SRC="$BAKED_SEED"
  echo "[bootstrap] Mounted /seed empty or missing; using baked fallback: $SEED_SRC"
else
  echo "[bootstrap] ERROR: No valid seed found. Checked: $MOUNT_SEED and $BAKED_SEED"
  exit 1
fi

echo "[bootstrap] Copying from $SEED_SRC to $DATA"
cp -a "$SEED_SRC"/. "$DATA"/

echo "[bootstrap] Setting ownership to 1000:1000 on $DATA"
chown -R 1000:1000 "$DATA"

# 3) Clone f1tenth_gym_ros if missing
GYM_DIR=/home/ubuntu/ros2_workspaces/src/f1tenth_gym_ros
if [ ! -d "$GYM_DIR" ] || [ -z "$(ls -A "$GYM_DIR" 2>/dev/null)" ]; then
    echo "[bootstrap] f1tenth_gym_ros not found, cloning..."
    git clone https://github.com/f1tenth/f1tenth_gym_ros.git "$GYM_DIR"
fi
chown -R 1000:1000 "$GYM_DIR"

echo "[bootstrap] Completed successfully"


# 4) Seed maps into f1tenth_gym_ros/maps once
MAPS_DIR="$GYM_DIR/maps"
MAPS_SENTINEL="$MAPS_DIR/.racer_maps_seeded"

if [ ! -f "$MAPS_SENTINEL" ]; then
    echo "[bootstrap] Seeding racetracks into $MAPS_DIR"
    mkdir -p "$MAPS_DIR"
    
    # need git in the bootstrap image (you already have it)
    rm -rf /tmp/tracks
    git clone --depth 1 https://github.com/f1tenth/f1tenth_racetracks.git /tmp/tracks
    
    for dir in /tmp/tracks/*/; do
        [ -d "$dir" ] || continue
        name=$(basename "$dir" | tr '[:upper:]' '[:lower:]')
        png=$(find "$dir" -maxdepth 1 -name "*.png" | head -1)
        yaml=$(find "$dir" -maxdepth 1 -name "*.yaml" | head -1)
        
        if [ -n "$png" ]; then
            cp "$png" "$MAPS_DIR/${name}.png"
        fi
        if [ -n "$yaml" ]; then
            cp "$yaml" "$MAPS_DIR/${name}.yaml"
            sed -i "s|image:.*|image: ${name}.png|" "$MAPS_DIR/${name}.yaml"
        fi
    done
    
    rm -rf /tmp/tracks
    chown -R 1000:1000 "$MAPS_DIR"
    touch "$MAPS_SENTINEL"
    chown 1000:1000 "$MAPS_SENTINEL"
    echo "[bootstrap] Maps seeded"
else
    echo "[bootstrap] Maps already seeded; skipping"
fi

# 5) Patch sim.yaml once
SIM_YAML="$GYM_DIR/config/sim.yaml"
SIM_SENTINEL="$GYM_DIR/config/.racer_sim_patched"

if [ -f "$SIM_YAML" ] && [ ! -f "$SIM_SENTINEL" ]; then
    echo "[bootstrap] Patching sim.yaml map_path"
    sed -i "s|map_path: .*|map_path: '/home/ubuntu/ros2_workspaces/src/f1tenth_gym_ros/maps/levine'|g" "$SIM_YAML"
    touch "$SIM_SENTINEL"
    chown 1000:1000 "$SIM_YAML" "$SIM_SENTINEL"
fi

echo "seeded_from=$SEED_SRC" > "$DATA/.racer_libs_seeded"

echo "[bootstrap] Completed successfully"
