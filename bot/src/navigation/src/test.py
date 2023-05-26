def normalize(angle):
        new_angle = angle
        if angle > 180:
            diff = angle -180
            new_angle = -180 +diff
        elif angle < -180:
            diff = angle + 180
            new_angle = 180 + diff
        return new_angle
    
def get_angle_ranges(orientation):
    ranges = {'straight_on': [-45,0,0,45],'go_back':[135,180,-180,-135],'right':[-135,-90,-90,-45],'left':[45,90,90,135]}
    commands = ['straight_on','go_back','right','left']
    for i in range(len(commands)):
        print(ranges[commands[i]][0])
        ranges[commands[i]][0] = normalize(ranges[commands[i]][0] + orientation)
        ranges[commands[i]][3] = normalize(ranges[commands[i]][3] + orientation)
        if ranges[commands[i]][0] == 135 and ranges[commands[i]][3] == -135:
            ranges[commands[i]][1] = 180
            ranges[commands[i]][2] = -180
        elif ranges[commands[i]][0] == -135 and ranges[commands[i]][3] == 135:
            ranges[commands[i]][1] = -180
            ranges[commands[i]][2] = 180
        else:
            ranges[commands[i]][1] = ranges[commands[i]][2] = (ranges[commands[i]][0] + ranges[commands[i]][3])/2
    return ranges


if __name__ == '__main__':
     print(get_angle_ranges(0))