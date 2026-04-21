import streamlit as st
import json
import os
import pandas as pd

# Set page config
st.set_page_config(page_title="LiteX Project Auditor", layout="wide")

@st.cache_data
def load_data():
    if not os.path.exists("audit_results.json"):
        return None
    with open("audit_results.json", "r") as f:
        return json.load(f)

data = load_data()

st.title("🔍 LiteX Project Auditor")

if data is None:
    st.error("Audit results not found. Please run `python3 tools/project_auditor.py` first.")
else:
    # --- Sidebar Navigation ---
    st.sidebar.header("Navigation")
    page = st.sidebar.radio("Go to", ["Dashboard", "Master File List", "File Search & Disambiguation", "Unused Files", "Target Explorer"])

    # --- Dashboard ---
    if page == "Dashboard":
        st.header("Project Overview")
        
        col1, col2, col3 = st.columns(3)
        
        total_hdl = len(data["hdl_usage"]) + len(data["unused_hdl"])
        total_py = len(data["py_usage"]) + len(data["unused_py"])
        
        col1.metric("Targets Tracked", len(data["targets"]))
        col2.metric("HDL Usage", f"{len(data['hdl_usage'])} / {total_hdl}", f"{len(data['unused_hdl'])} unused", delta_color="inverse")
        col3.metric("Python Usage", f"{len(data['py_usage'])} / {total_py}", f"{len(data['unused_py'])} unused", delta_color="inverse")

        st.subheader("Target Distribution")
        target_stats = []
        for target, files in data["targets"].items():
            target_stats.append({
                "Target": target,
                "HDL Files": len(files["hdl"]),
                "Python Files": len(files["py"])
            })
        st.dataframe(pd.DataFrame(target_stats), width="stretch")

    # --- Master File List ---
    elif page == "Master File List":
        st.header("Master Source File List")
        st.info("A comprehensive list of all source files found in the project (excluding build/deps).")
        
        type_filter = st.radio("File Type", ["HDL", "Python"], key="master_type")
        
        if type_filter == "HDL":
            master_list = data.get("master_hdl", [])
            usage_map = data["hdl_usage"]
        else:
            master_list = data.get("master_py", [])
            usage_map = data["py_usage"]

        master_df = []
        for item in master_list:
            if isinstance(item, dict):
                path = item["path"]
                is_used = item["used"]
                targets = item.get("targets", usage_map[path]["targets"] if path in usage_map else [])
                callers = item.get("callers", usage_map[path]["callers"] if path in usage_map else [])
            else:
                # Fallback for old data structure
                path = item
                is_used = path in usage_map
                targets = usage_map[path]["targets"] if is_used else []
                callers = usage_map[path]["callers"] if is_used else []

            master_df.append({
                "Path": path,
                "Status": "✅ USED" if is_used else "⚠️ UNUSED",
                "Targets": ", ".join(targets),
                "Callers": ", ".join(callers),
                "Targets Count": len(targets),
                "Callers Count": len(callers)
            })
        
        if not master_df and not master_list:
            # Complete fallback if master_hdl/py is missing from data
            fallback_list = sorted(list(set(usage_map.keys()) | set(data["unused_hdl" if type_filter == "HDL" else "unused_py"])))
            for path in fallback_list:
                is_used = path in usage_map
                master_df.append({
                    "Path": path,
                    "Status": "✅ USED" if is_used else "⚠️ UNUSED",
                    "Targets Count": len(usage_map[path]["targets"]) if is_used else 0,
                    "Callers Count": len(usage_map[path]["callers"]) if is_used else 0
                })
        
        df = pd.DataFrame(master_df)
        
        col1, col2 = st.columns([2, 1])
        with col2:
            status_filter = st.multiselect("Filter by Status", ["✅ USED", "⚠️ UNUSED"], default=["✅ USED", "⚠️ UNUSED"])
        
        filtered_df = df[df["Status"].isin(status_filter)]
        
        search_master = st.text_input("Search master list...", "")
        if search_master:
            filtered_df = filtered_df[filtered_df["Path"].str.contains(search_master, case=False)]
            
        st.dataframe(filtered_df, width="stretch", height=600)

    # --- File Search & Disambiguation ---
    elif page == "File Search & Disambiguation":
        st.header("Search & Disambiguation")
        st.info("Identify if multiple files with the same name are used, and where.")
        
        search_query = st.text_input("Search by filename (e.g. 'top.vhd')", "").strip()
        
        if search_query:
            # Gather all files (HDL + PY) matching the name
            all_usage = {**data["hdl_usage"], **data["py_usage"]}
            all_unused = data["unused_hdl"] + data["unused_py"]
            
            matches = []
            
            # Check used files
            for path, info in all_usage.items():
                if search_query.lower() in os.path.basename(path).lower():
                    matches.append({
                        "Path": path,
                        "Status": "✅ USED",
                        "Targets": ", ".join(info["targets"]),
                        "Callers": ", ".join(info["callers"])
                    })
            
            # Check unused files
            for path in all_unused:
                if search_query.lower() in os.path.basename(path).lower():
                    matches.append({
                        "Path": path,
                        "Status": "⚠️ UNUSED",
                        "Targets": "-",
                        "Callers": "-"
                    })
            
            if matches:
                st.write(f"Found {len(matches)} matches for '{search_query}':")
                st.table(pd.DataFrame(matches))
            else:
                st.warning("No files found matching that name.")

    # --- Unused Files ---
    elif page == "Unused Files":
        st.header("Audit: Unused Files")
        
        type_filter = st.radio("File Type", ["HDL", "Python"])
        
        if type_filter == "HDL":
            unused_list = data["unused_hdl"]
        else:
            unused_list = data["unused_py"]
            
        st.write(f"Found {len(unused_list)} unused {type_filter} files.")
        
        if unused_list:
            search_unused = st.text_input("Filter unused list...", "")
            filtered_unused = [f for f in unused_list if search_unused.lower() in f.lower()]
            st.dataframe(pd.DataFrame(filtered_unused, columns=["Path"]), width="stretch", height=600)
            
            if st.button("Copy list to clipboard"):
                st.write("Feature not available in web interface, please copy from table above.")

    # --- Target Explorer ---
    elif page == "Target Explorer":
        st.header("Target Dependency Explorer")
        
        selected_target = st.selectbox("Select a Board Target", sorted(data["targets"].keys()))
        
        if selected_target:
            target_data = data["targets"][selected_target]
            
            tab1, tab2 = st.tabs(["HDL Files", "Python Files"])
            
            with tab1:
                st.write(f"Files used by {selected_target}:")
                hdl_df = []
                for path in target_data["hdl"]:
                    callers = data["hdl_usage"].get(path, {}).get("callers", [])
                    hdl_df.append({"Path": path, "Callers": ", ".join(callers)})
                st.dataframe(pd.DataFrame(hdl_df), width="stretch")
                
            with tab2:
                st.write(f"Modules imported by {selected_target}:")
                py_df = []
                for path in target_data["py"]:
                    callers = data["py_usage"].get(path, {}).get("callers", [])
                    py_df.append({"Path": path, "Callers": ", ".join(callers)})
                st.dataframe(pd.DataFrame(py_df), width="stretch")

# Footer
st.divider()
st.caption("LiteX Project Auditor - Generated Data")
