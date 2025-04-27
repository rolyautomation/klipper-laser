# Generate supergcode
#
# Copyright (C) 2025-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, re


class SGcodeItemData:
    def __init__(self, coord_state, gtype, linestr):
        self.coord = coord_state
        self.gtype = gtype
        self.axis_flags = 0
        self.curpos = [0.0] * 7
        self.curmd = [0.0] *  7
        self.angle = 0.0
        self.instr = linestr

class GenSuperGcode:
    def __init__(self, config):
        self.printer = config.get_printer()
        #self.name = config.get_name().split(' ')[-1]
        self.epsilon = 0.0001

        self.blocklen = config.getint('blocklen', 8, minval=2, maxval=24)
        self.enable = config.getint('enable', 1, minval=0, maxval=1)
        self.AXIS_X = 0x1
        self.AXIS_Y = 0x2
        self.AXIS_Z = 0x4
        self.AXIS_A = 0x8
        self.AXIS_B = 0x10
        self.AXIS_C = 0x20
        self.PARAM_F = 0x40
        self.PARAM_S = 0x80    
        self.absolute_coord = True  # G90为True，G91为False   

        self.cached_cmds = []
        self.path_points = []  # 存储路径点信息
        self.last_axis_flags = 0
        self.last_abs_pos = [-1.0] * 7
        self.abs_pos_bound = 0
        self.last_speed = 0.0
        self.last_power = 0.0
        #self.last_angle = 0.0
        
        self.gcode = self.printer.lookup_object('gcode')

    args_r = re.compile('([A-Z_]+|[A-Z*/])')
    def parse_gcode(self, line, coord_state, gtype):
        # 解析G0/G1指令
        itemdata = SGcodeItemData(coord_state, gtype, line)
        parts = self.args_r.split(line)
        numparts = len(parts)
        params = {parts[i]: float(parts[i+1].strip())
                  for i in range(1, numparts, 2)}
        #if line.startswith('G0') or line.startswith('G1'):
        for param, value in params.items():
            try:
                if param == 'X': 
                    itemdata.axis_flags |= self.AXIS_X
                    if coord_state:
                        itemdata.curpos[0] = value
                        if self.last_abs_pos[0] > self.abs_pos_bound:
                            itemdata.curmd[0] =  value - self.last_abs_pos[0]
                        self.last_abs_pos[0] = value
                    else:
                        itemdata.curmd[0] = value 
                elif param == 'Y': 
                    itemdata.axis_flags |= self.AXIS_Y
                    if coord_state:
                        itemdata.curpos[1] = value
                        if self.last_abs_pos[1] > self.abs_pos_bound:
                            itemdata.curmd[1] =  value - self.last_abs_pos[1]
                        self.last_abs_pos[1] = value                            
                    else:
                        itemdata.curmd[1] = value                         
                elif param == 'Z': 
                    itemdata.axis_flags |= self.AXIS_Z
                    if coord_state:
                        itemdata.curpos[2] = value
                        if self.last_abs_pos[2] > self.abs_pos_bound:
                            itemdata.curmd[2] =  value - self.last_abs_pos[2]
                        self.last_abs_pos[2] = value                            
                    else:
                        itemdata.curmd[2] = value                           
                elif param == 'A': 
                    itemdata.axis_flags |= self.AXIS_A
                    if coord_state:
                        itemdata.curpos[3] = value
                        if self.last_abs_pos[3] > self.abs_pos_bound:
                            itemdata.curmd[3] =  value - self.last_abs_pos[3]
                        self.last_abs_pos[3] = value                            
                    else:
                        itemdata.curmd[3] = value                           
                elif param == 'B': 
                    itemdata.axis_flags |= self.AXIS_B
                    if coord_state:
                        itemdata.curpos[4] = value
                        if self.last_abs_pos[4] > self.abs_pos_bound:
                            itemdata.curmd[4] =  value - self.last_abs_pos[4]
                        self.last_abs_pos[4] = value                            
                    else:
                        itemdata.curmd[4] = value                           
                elif param == 'C': 
                    itemdata.axis_flags |= self.AXIS_C
                    if coord_state:
                        itemdata.curpos[5] = value
                        if self.last_abs_pos[5] > self.abs_pos_bound:
                            itemdata.curmd[5] =  value - self.last_abs_pos[5]
                        self.last_abs_pos[5] = value                            
                    else:
                        itemdata.curmd[5] = value                           
                elif param == 'F': 
                    itemdata.axis_flags |= self.PARAM_F
                    itemdata.curmd[6] = value 
                    self.last_speed = value
                elif param == 'S': 
                    itemdata.axis_flags |= self.PARAM_S
                    itemdata.curmd[7] = value 
                    self.last_power = value
            except ValueError:
                continue
        return itemdata
        
    def merge_gcode_in_cache(self):
        new_line = ""
        if not self.cached_cmds:
            return ""
        #todo

            
        return new_line

    def is_straight_line(self, itemdata):
        bstatus = False
        if self.cached_cmds[-1].axis_flags == itemdata.axis_flags:
            bstatus = True
        #todo            
        return bstatus

        
    def check_flush_cache(self, absolute_coord_state):
        new_line = ""
        if self.cached_cmds:
            if len(self.cached_cmds) == 1:
                new_line = self.cached_cmds[0].instr
            else:
                new_line = self.merge_gcode_in_cache()
            self.cached_cmds = []
        return new_line


    def is_merge_gcode(self, itemdata):
        statuscode =  1
        new_line_str = ""
        if not self.cached_cmds:
            self.cached_cmds.append(itemdata)
            return 0, new_line_str

        if self.is_straight_line(itemdata):
            self.cached_cmds.append(itemdata)
            statuscode = 0
            if len(self.cached_cmds) == self.blocklen:
                new_line_str = self.check_flush_cache(self.absolute_coord)
                if new_line_str:
                    statuscode = 2
            return statuscode, new_line_str
        else:
            new_line_str = self.check_flush_cache(self.absolute_coord)
            if new_line_str:
                statuscode = 2
            self.cached_cmds.append(itemdata)
            return statuscode, new_line_str
        return statuscode, new_line_str


    def clean_cache_cmd(self):
        self.cached_cmds = []
        self.last_axis_flags = 0
        self.last_abs_pos = [-1.0] * 7
        self.last_speed = 0.0
        self.last_power = 0.0        
        pass



    def handle_supercode(self, line_gcode):
        retstatus = 0
        linestr = ""
        statuscode =  1
        if self.enable == 0:
            return statuscode, line_gcode

        linestr = line_gcode.strip().upper()
        if not linestr or linestr.startswith(';'):
            return statuscode, linestr
            
        if 'G90' in linestr:
            self.absolute_coord = True
            new_line = self.check_flush_cache(self.absolute_coord)
            if new_line:
                statuscode = 2
            return statuscode, new_line + linestr
        if 'G91' in linestr:
            self.absolute_coord = False
            new_line = self.check_flush_cache(self.absolute_coord)
            if new_line:
                statuscode = 2            
            return statuscode, new_line + linestr     

        if  line.startswith('G1'):
            itemdata = self.parse_gcode(linestr, self.absolute_coord, 1)
            statuscode, new_line = self.is_merge_gcode(itemdata)
            return statuscode,new_line
        elif  line.startswith('G0'):
            itemdata = self.parse_gcode(linestr, self.absolute_coord, 0)  
            new_line = self.check_flush_cache(self.absolute_coord)
            if new_line:
                statuscode = 2
            return statuscode, new_line + line_gcode
        else:               
            new_line = self.check_flush_cache(self.absolute_coord)
            if new_line:
                statuscode = 2
            return statuscode, new_line + line_gcode

    def get_status(self, eventtime=None):
        supergcode_status = {}
        supergcode_status['blocklen'] = self.blocklen
        supergcode_status['enable'] = self.enable   
        return dict(supergcode_status)


def load_config(config):
    return GenSuperGcode(config)
#self.gen_supergcode_obj = self.printer.lookup_object('gen_supergcode',None)
