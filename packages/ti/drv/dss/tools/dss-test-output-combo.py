import itertools
import sys

variations = { 

# Different ways you can use dpi-0
# * Disabled
# * Single-video-port mode, for RGB (with discrete sync), BT656, BT1120
#   - using VP2
#   - using VP4
# * Dual-video-port mode, for YUV422 discrete sync
#   - using VP2 for data and VP1 for sync
#   - using VP4 for data and VP3 for sync
"dpi-0" : ["disabled", "vp2", "vp4", "ds-vp1-vp2", "ds-vp3-vp4"],

# Different ways you can use dpi-1
# * Disabled
# * Single-video-port mode, for RGB (with discrete sync), BT656, BT1120
#   - using VP2
#   - using VP4
# * Dual-video-port mode, for YUV422 discrete sync
#   Note: Cannot use VP3/V4 with DPI-1 for pinmux and MSS restrictions
#   - using VP2 for data and VP1 for sync
"dpi-1" : ["disabled", "vp2", "vp4", "ds-vp1-vp2"],

# Different ways you can use display port
# * Disabled
# * SST mode
#   - using VP1
"dp" : ["disabled", "sst-vp1"],

# Different ways you can use dsi
# * Disabled
# * Using VP3
"dsi" : ["disabled", "vp3"],

}

# return number of times a video-port  is used for a combo
def create_vp_map(combo):
	vp_map = {
		"vp1"        : (1, 0, 0, 0),
		"vp2"        : (0, 1, 0, 0),
		"vp3"        : (0, 0, 1, 0),
		"vp4"        : (0, 0, 0, 1),
		"ds-vp3-vp4" : (0, 0, 1, 1),	
		"ds-vp1-vp2" : (1, 1, 0, 0),
		"sst-vp1"    : (1, 0, 0, 0),
	}
	combo_vp_map = (0, 0, 0, 0)
	for st in combo:
		st_name = st.split("_")[1]
		if st_name == "disabled":
			combo_vp_map = tuple(map(sum, zip(combo_vp_map, (0, 0, 0, 0))))
		else:
			combo_vp_map = tuple(map(sum, zip(combo_vp_map, vp_map[st_name])))
	return combo_vp_map


# see if a pclk is used, and return false if not same. If not used yet, use it 
def update_final_pclk(pclk_map, final_setup, key):
	if not key in pclk_map:
		return True

	if key in final_setup:
		if final_setup[key] == pclk_map[key]:
			return True
		else:
			return False
	
	final_setup[key] = pclk_map[key]
	return True
	
# see if request mux setup is subset of what is currently in use
# If subset, keep the intersect as current
# Else, reject this mux setup request
def update_final_mux(mux_map, final_setup, key):
	if not key in mux_map:
		return True

	if not key in final_setup:
		final_setup[key] = mux_map[key]
		return True
	
	intersect = set(mux_map[key]) & set(final_setup[key])
	if intersect:
		final_setup[key] = list(intersect)
		return True

	return False

# See if combo is possible by thrashing all standalone pclk and mux settings
# against one-another
def mux_pclk_test(mux_map, pclk_map, combo_list):
	final_setup = {}


	for st in combo_list:
		mux_list  = mux_map[st[0]]
		pclk_list = pclk_map[st[0]]

		mux  = mux_list[st[2]]
		pclk = pclk_list[st[2]]

		if not update_final_pclk(pclk, final_setup, "pll-16-0"):
			return None, False
		if not update_final_pclk(pclk, final_setup, "pll-16-1"):
			return None, False
		if not update_final_pclk(pclk, final_setup, "pll-17-0"):
			return None, False
		if not update_final_pclk(pclk, final_setup, "pll-17-1"):
			return None, False
		if not update_final_pclk(pclk, final_setup, "pll-18-0"):
			return None, False
		if not update_final_pclk(pclk, final_setup, "pll-18-1"):
			return None, False
		if not update_final_pclk(pclk, final_setup, "pll-19-0"):
			return None, False
		if not update_final_pclk(pclk, final_setup, "pll-23-0"):
			return None, False

		if not update_final_mux(mux, final_setup, "dpi-1-pclk-sel"):
			return None, False
		if not update_final_mux(mux, final_setup, "dpi-2-pclk-sel"):
			return None, False
		if not update_final_mux(mux, final_setup, "dpi-3-pclk-sel"):
			return None, False

	return final_setup, True 

#rotate a list of tuples using tuple element 2 (max in tuple element 1)
def rotate_clist(combo_list, index):
	combo_list[index] = (combo_list[index][0], combo_list[index][1], (combo_list[index][2] + 1) % combo_list[index][1])
	if combo_list[index][2] == 0:
		if index < len(combo_list) - 1:
			rotate_clist(combo_list, index + 1)
		
def validate_mux_pclk_map(combo, clk):
	allowed_mux_map = {
		"dpi-0_vp2"        : [{"dpi-1-pclk-sel" : [1]}],
		"dpi-0_vp4"        : [{"dpi-3-pclk-sel" : [3]}],
		"dpi-1_vp2"        : [{"dpi-1-pclk-sel" : [2]}],
		"dpi-1_vp4"        : [{"dpi-3-pclk-sel" : [4, 5, 6, 7]}],
		"dpi-0_ds-vp1-vp2" : [{"dpi-1-pclk-sel" : [1], "dpi-3-pclk-sel" : [5, 7]}],
		"dpi-0_ds-vp3-vp4" : [{"dpi-3-pclk-sel" : [6, 7]}],
		"dpi-1_ds-vp1-vp2" : [{"dpi-1-pclk-sel" : [2], "dpi-3-pclk-sel" : [5, 7]}],
		"dp_sst-vp1"       : [{"dpi-3-pclk-sel" : [0, 1, 2, 3, 4, 6]},
		                      {"dpi-1-pclk-sel" : [0], "dpi-3-pclk-sel" : [5, 7]},
		                      {"dpi-1-pclk-sel" : [1], "dpi-3-pclk-sel" : [5, 7]},
		                      {"dpi-1-pclk-sel" : [2], "dpi-3-pclk-sel" : [5, 7]},
		                      {"dpi-1-pclk-sel" : [3], "dpi-3-pclk-sel" : [5, 7]}],
		"dsi_vp3"          : [{"dpi-2-pclk-sel" : [1], "dpi-3-pclk-sel" : [0, 1, 2, 3, 4, 5]},
		                      {"dpi-2-pclk-sel" : [0], "dpi-3-pclk-sel" : [0, 1, 2, 3, 4, 5]},
				      {"dpi-3-pclk-sel" : [6, 7]}],
	}
	allowed_pclk_map = {
		"dpi-0_vp2"        : [{"pll-19-0" : clk["dpi-0"]}],
		"dpi-0_vp4"        : [{"pll-19-0" : clk["dpi-0"]}],
		"dpi-1_vp2"        : [{"pll-23-0" : clk["dpi-1"]}],
		"dpi-1_vp4"        : [{"pll-23-0" : clk["dpi-1"]}],
		"dpi-0_ds-vp1-vp2" : [{"pll-19-0" : clk["dpi-0"]}],
		"dpi-0_ds-vp3-vp4" : [{"pll-19-0" : clk["dpi-0"]}],
		"dpi-1_ds-vp1-vp2" : [{"pll-23-0" : clk["dpi-1"]}],
		"dp_sst-vp1"       : [{"pll-16-0" : clk["dp"]},
		                      {"pll-17-0" : clk["dp"]},
		                      {"pll-19-0" : clk["dp"]},
		                      {"pll-23-0" : clk["dp"]},
		                      {"pll-16-0" : clk["dp"]}],
		"dsi_vp3"          : [{"pll-18-0" : clk["dsi"]},
		                      {"pll-16-0" : clk["dsi"]},
				      {"pll-19-0" : clk["dsi"]}],
	}

	# create a list of enabled vouts
	clist = []
	for st in combo:
		st_name = st.split("_")[1]
		if st_name != "disabled":
			clist.append((st, len(allowed_mux_map[st]), 0))

	# if none enabled, we support it :) 
	if len(clist) < 1:
		return {}, True 

	# test for all pclk variants in this combo, return whenever a hit is found
	found = False
	while True:
		setup, flag = mux_pclk_test(allowed_mux_map, allowed_pclk_map, clist)
		if flag:
			return setup, True
		
		rotate_clist(clist, 0)
		if(all(elem[2] == 0 for elem in clist)):
			return None, False

# See if any video-port is repeated
# else validate mux and pclk settings
def validate(combo, clk):
	print combo
	print type(combo)
	if(all(nvp < 2 for nvp in create_vp_map(combo))):
		setup, flag = validate_mux_pclk_map(combo, clk)
		if flag:	
			return setup, True
	return None, False


def pclk(setup, key):
	if not key in setup:
		return ""
	return key + " = " + str(setup[key]) + ", "

def muxval(setup, key):
	outstr = ""
	if not key in setup:
		return ""
	for val in setup[key]:
		outstr += " " + str(val) + " or"
	outstr = outstr[:-3]
	outstr = outstr[1:]

	return key + " = " + outstr + ", "


def pretty_print(combo, setup):
	print_first = True;
	outstr = "Combo: "

	for st in combo:
		st_name = st.split("_")[1]
		st_head = st.split("_")[0]
		if st_name != "disabled":
			if not print_first:
				outstr += ", " + st_head + "(" + st_name + ")"
			else:
				outstr += st_head + "(" + st_name + ")"
				print_first = False
	
	outstr += " using:\n    clocks: "

	outstr += pclk(setup, "pll-16-0")
	outstr += pclk(setup, "pll-16-1")
	outstr += pclk(setup, "pll-17-0")
	outstr += pclk(setup, "pll-17-1")
	outstr += pclk(setup, "pll-18-0")
	outstr += pclk(setup, "pll-18-1")
	outstr += pclk(setup, "pll-19-0")
	outstr += pclk(setup, "pll-23-0")

	outstr = outstr[:-2] + "\n    mux-values: "
	outstr += muxval(setup, "dpi-1-pclk-sel")
	outstr += muxval(setup, "dpi-2-pclk-sel")
	outstr += muxval(setup, "dpi-3-pclk-sel")

	outstr = outstr[:-2]

	print outstr

def extract_supersets(combo_list):
	combos = []
	super_combo = []

	for combo, setup in combo_list:
		mini = []
		for st in combo:
			st_name = st.split("_")[1]
			if st_name != "disabled":
				mini.append(st)
		combos.append((set(mini), setup))

	for cset, setup in combos:
		if cset:
			found = False
			for compset, dummy in combos:
				if compset and not cset is compset:
					if cset.issubset(compset):
						found = True;
						break;
			if not found:
				super_combo.append((cset, setup))


	return super_combo
			


def distinct_clock_variations(superset):
	combo_list = []

	for combo in itertools.product(
			map(lambda orig_string: "dpi-0_" + orig_string, variations["dpi-0"]),
			map(lambda orig_string: "dpi-1_" + orig_string, variations["dpi-1"]),
			map(lambda orig_string: "dp_" + orig_string, variations["dp"]),
			map(lambda orig_string: "dsi_" + orig_string, variations["dsi"])
			):
		setup, flag = validate(combo, {"dpi-0" : 1, "dpi-1" : 2, "dp" : 3, "dsi" : 4})
		if flag:
			combo_list.append((combo, setup))

	if superset:
		super_combo_list = extract_supersets(combo_list)
		for (combo, setup) in super_combo_list:
			pretty_print(combo, setup)
	else:
		for (combo, setup) in combo_list:
			pretty_print(combo, setup)

distinct_clock_variations(True)
