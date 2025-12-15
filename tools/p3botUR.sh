#!/bin/bash

# ===========================
# VERIFICACIÓN DE VARIABLES
# ===========================
if [ -z "$ROBOCOMP" ]; then

    echo "ROBOCOMP environment variable not set, using the default value /home/robocomp/robocomp"
    ROBOCOMP="/home/robocomp/robocomp"
fi

# ===========================
# CONFIGURACIÓN DE COMPONENTES
# ===========================
COMPILE="cmake -B build && make -C build -j$(( $(nproc) / 2 ))"

declare -A COMPONENTS=(
    ["RcNode"]="rcnode"
    ["Ricoh"]="cd $ROBOCOMP/components/robocomp-robolab/components/hardware/camera/ricoh_omni && $COMPILE && bin/RicohOmni etc/config"
    ["Helios"]="cd $ROBOCOMP/components/robocomp-robolab/components/hardware/laser/lidar3D && $COMPILE && bin/Lidar3D etc/config_p3bot"
    ["360 fusion"]="cd $ROBOCOMP/components/robocomp-shadow/insect/RGBD_360 && $COMPILE && bin/RGBD_360 etc/config_UR"
    ["ZED"]="cd $ROBOCOMP/components/robocomp-shadow/insect/zed_component && $COMPILE && bin/zed_component etc/etc/configP3bot.toml"
    ["kinova_left"]="cd $ROBOCOMP/components/manipulation_kinova_gen3/agents/kinova_controller_cpp && $COMPILE && bin/kinova_controller_cpp etc/config_brazo_pablo"
    ["kinova_right"]="cd $ROBOCOMP/components/manipulation_kinova_gen3/agents/kinova_controller_cpp && $COMPILE && bin/kinova_controller_cpp etc/config_brazo_pedro"
    ["Body_control"]="cd $ROBOCOMP/components/robocomp-p3bot/components/armController && $COMPILE && bin/armController etc/config_real"
    
)

function show_tabs() {
    echo "=== List of existing tabs ==="
    session_ids=($(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.sessionIdList | tr ',' ' '))
    if [ $? -ne 0 ]; then
        echo "Error: Failed to get session IDs"
        return 1
    fi

    for session_id in "${session_ids[@]}"; do
        tab_title=$(qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.tabTitle "$session_id")
        if [ $? -ne 0 ]; then
            echo "Error: Failed to get tab title for session ID $session_id"
            continue
        fi

        echo "Tab ID: $session_id, Title: $tab_title"
    done
}

# ===========================
# ELIMINAR PESTAÑAS Y PROCESOS EXISTENTES
# ===========================
function clean_tabs() {
    echo "=== Cleaning tabs ==="
    session_ids=($(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.sessionIdList | tr ',' ' '))
    if [ $? -ne 0 ]; then
        echo "Error: Failed to get session IDs"
        return 1
    fi

    for session_id in "${session_ids[@]}"; do
        tab_title=$(qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.tabTitle "$session_id")
        if [ $? -ne 0 ]; then
            echo "Error: Failed to get tab title for session ID $session_id"
            continue
        fi

        if [[ -n "${COMPONENTS[$tab_title]}" ]]; then
            # Extraer el nombre del ejecutable del comando
            command_parts=(${COMPONENTS[$tab_title]})
            last_part=${command_parts[-1]}
            process_name=$(basename "$last_part" | cut -d' ' -f1)

            # Kill the running process
            qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "killall -9 $process_name" 2>/dev/null
            
            # Remove the tab
            qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.removeSession "$session_id"
            if [ $? -ne 0 ]; then
                echo "Error: Failed to remove tab with title $tab_title"
                continue
            fi
            echo "Removed tab with title: $tab_title"
        fi
    done
}

function start_agents() {
    echo "=== Starting agents ==="
    for TAB_NAME in "${!COMPONENTS[@]}"; do
        COMMAND="${COMPONENTS[$TAB_NAME]}"
        session_id=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
        if [ $? -ne 0 ]; then
            echo "Error: Failed to create new tab for $TAB_NAME"
            continue
        fi
        qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle "$session_id" "$TAB_NAME"
        qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "$COMMAND"
        qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.raiseSession $session_id
    done
}

# ===========================
# MAIN
# ===========================
if [ "$1" == "stop" ]; then
    clean_tabs
elif [ "$1" == "list" ]; then
    show_tabs
else
    clean_tabs
    start_agents
fi

echo "=== Script completado ==="
