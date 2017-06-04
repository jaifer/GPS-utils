# Convert NMEA messages to csv format

import glob
import math
import utm

FILES = glob.glob('*.txt')

VTG_dir = []
VTG_speed = []
GGA_time = []
GGA_lat = []
GGA_lon = []
GGA_quality = []
GGA_num = []
GGA_dop = []
GLL_time = []
GLL_lat = []
GLL_lon = []
GLL_valid = []
RMC_time = []
RMC_lat = []
RMC_lon = []
RMC_valid = []
RMC_dir = []
RMC_speed = []

#WGS 84
a = 6378137.0 #DatumEqRad
f = 1.0/298.2572236 #DatumFlat
k0 = 0.9996 #scale on central meridian
b = a*(1-f) #polar axis
e = math.sqrt(1-(b/a)*(b/a)) #eccentricity
drad = math.pi/180.0
phi = 0 #latitude (north +, south -), but uses phi in reference
e0 = e/math.sqrt(1 - e*e) # e prime in reference

latd = 0 #latitude in degrees
N = a/math.sqrt(1-math.pow(e*math.sin(phi),2))
T = math.pow(math.tan(phi),2)
C = math.pow(e*math.cos(phi),2)
lng = 0#;//Longitude (e = +, w = -) - can't use long - reserved word
lng0 = 0#;//longitude of central meridian
lngd = 0#;//longitude in degrees
M = 0#;//M requires calculation
x = 0#;//x coordinate
y = 0#;//y coordinate
k = 1#;//local scale
utmz = 30#;//utm zone
zcm = 0#;//zone central meridian
DigraphLetrsE = "ABCDEFGHJKLMNPQRSTUVWXYZ"
DigraphLetrsN = "ABCDEFGHJKLMNPQRSTUV"

def LatLonToUTM(lat, lon):
    if ((lat == '') or (lon == '')):
        return 0, 0, 0
    lat_d = lat.split('.')
    lon_d = lon.split('.')
    latd = int(lat_d[0][:-2]) + int(lat_d[0][-2:])/60.0 + float('0.'+lat_d[1][:-1])/60.0
    lngd = int(lon_d[0][:-2]) + int(lon_d[0][-2:])/60.0 + float('0.'+lon_d[1][:-1])/60.0
    if (lat[-1] == 'S'):
        latd = (-1)*latd
    if (lon[-1] == 'W'):
        lngd = (-1)*lngd

    EASTING, NORTHING, ZONE_NUM, ZONE_LETTER = utm.from_latlon(latd, lngd)
    return ZONE_NUM, EASTING, NORTHING

# http://www.uwgb.edu/dutchs/UsefulData/ConvertUTMNoOZ.HTM
# Convert Latitude and Longitude to UTM
def GeogToUTM(lat, lon):
    # lat = 4807.038N   Latitude 48 deg 07.038' N
    # lon = 01131.000W  Longitude 11 deg 31.000' W
    if ((lat == '') or (lon == '')):
        return 0, 0, 0
    lat_d = lat.split('.')
    lon_d = lon.split('.')
    latd = int(lat_d[0][:-2]) + int(lat_d[0][-2:])/60.0 + float('0.'+lat_d[1][:-1])/60.0
    lngd = int(lon_d[0][:-2]) + int(lon_d[0][-2:])/60.0 + float('0.'+lon_d[1][:-1])/60.0
    if (lat[-1] == 'S'):
        latd = (-1)*latd
    if (lon[-1] == 'W'):
        lngd = (-1)*lngd

    if ((latd <-90) or (latd> 90)):
        print("Latitude must be between -90 and 90")
        quit()
    if ((lngd <-180) or (lngd > 180)):
        print("Latitude must be between -180 and 180")

    phi = latd*drad #;//Convert latitude to radians
    lng = lngd*drad #;//Convert longitude to radians
    utmz = 1 + math.floor((lngd+180)/6) #;//calculate utm zone
    latz = 0 #;//Latitude zone: A-B S of -80, C-W -80 to +72, X 72-84, Y,Z N of 84
    if ((latd > -80) and (latd < 72)):
        latz = math.floor((latd + 80)/8)+2
    if ((latd > 72) and (latd < 84)):
        latz = 21
    if (latd > 84):
        latz = 23
        
    zcm = 3 + 6*(utmz-1) - 180 #;//Central meridian of zone

    #Calculate Intermediate Terms
    esq = (1 - (b/a)*(b/a)) #;//e squared for use in expansions
    e0sq = e*e/(1-e*e) #;// e0 squared - always even powers
    N = a/math.sqrt(1-math.pow(e*math.sin(phi),2))
    T = math.pow(math.tan(phi),2)
    C = e0sq*math.pow(math.cos(phi),2)
    A = (lngd-zcm)*drad*math.cos(phi)
    M = phi*(1 - esq*(1/4 + esq*(3/64 + 5*esq/256)))
    M = M - math.sin(2*phi)*(esq*(3/8 + esq*(3/32 + 45*esq/1024)))
    M = M + math.sin(4*phi)*(esq*esq*(15/256 + esq*45/1024))
    M = M - math.sin(6*phi)*(esq*esq*esq*(35/3072))
    M = M*a #;//Arc length along standard meridian
    M0 = 0 #;//M0 is M for some origin latitude other than zero. Not needed for standard UTM

    # Calculate UTM Values
    x = k0*N*A*(1 + A*A*((1-T+C)/6.0 + A*A*(5 - 18*T + T*T + 72*C -58*e0sq)/120.0)) #;//Easting relative to CM
    x=x+500000 #;//Easting standard 
    y = k0*(M - M0 + N*math.tan(phi)*(A*A*(1.0/2.0 + A*A*((5 - T + 9*C + 4*C*C)/24.0 + A*A*(61 - 58*T + T*T + 600*C - 330*e0sq)/720.0)))) #;//Northing from equator
    yg = y + 10000000 #;//yg = y global, from S. Pole
    if (y < 0):
        y = 10000000+y

    ## Output into UTM Boxes
    zone = utmz
    easting = math.floor(10*x)/10 # meters
    northing = math.floor(10*y)/10 # meters
    if (phi<0):
        south =true
    return zone, easting, northing
    # NATO UTM
#       LonZone = utmz;
#       LatZone = DigraphLetrsE[latz];
#       easting = Math.round(10*(x-100000*Math.floor(x/100000)))/10;
#       northing = Math.round(10*(y-100000*Math.floor(y/100000)))/10;
#       MakeDigraph();
#       document.getElementById("UTMDgBox2").value = Digraph;

#Velocity made good.
#$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
#      VTG      Track made good and ground speed
#      054.7,T      True track made good (degrees)
#      034.4,M      Magnetic track made good
#      005.5,N      Ground speed, knots
#      010.2,K      Ground speed, Kilometers per hour
#      *48      Checksum
def ProcVTG(data):
    VTG_dir.append(data[1])
    VTG_speed.append(data[7])
    return ''

#$--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx
#      GGA      Global Positioning System Fix Data
#      hhmmss.ss = UTC of position
#      llll.ll = latitude of position
#      a = N or S
#      yyyyy.yy = Longitude of position
#      a = E or W
#      x = GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
#      xx = number of satellites in use
#      x.x = horizontal dilution of precision (smaller is better)
#      x.x = Antenna altitude above mean-sea-level
#      M = units of antenna altitude, meters
#      x.x = Geoidal separation
#      M = units of geoidal separation, meters
#      x.x = Age of Differential GPS data (seconds)
#      xxxx = Differential reference station ID 
def ProcGGA(data):
    GGA_time.append(data[1])
    GGA_lat.append(data[2] + data[3])
    GGA_lon.append(data[4] + data[5])
    GGA_quality.append(data[6])
    GGA_num.append(data[7])
    GGA_dop.append(data[8])
    return ''

#$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
#     GSA      Satellite status
#     A    Auto selection of 2D or 3D fix (M = manual) 
#     3    3D fix - values include: 1 = no fix
#                       2 = 2D fix
#                       3 = 3D fix
#     04,05... PRNs of satellites used for fix (space for 12) 
#     2.5      PDOP (dilution of precision) 
#     1.3      Horizontal dilution of precision (HDOP) 
#     2.1      Vertical dilution of precision (VDOP)
def ProcGSA(data):
    return ''

#$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75
#      GSV      Satellites in view
#      2        Number of sentences for full data
#      1        sentence 1 of 2
#      08       Number of satellites in view
#      01       Satellite PRN number
#      40       Elevation, degrees
#      083      Azimuth, degrees
#      46       SNR - higher is better
#       for up to 4 satellites per sentence
def ProcGSV(data):
    return ''

# $GPGLL,4916.45,N,12311.12,W,225444,A,*1D
#     GLL      Geographic position, Latitude and Longitude
#     4916.46,N    Latitude 49 deg. 16.45 min. North
#     12311.12,W   Longitude 123 deg. 11.12 min. West
#     225444       Fix taken at 22:54:44 UTC
#     A        Data Active or V (void)
#     *iD      checksum data
def ProcGLL(data):
    GLL_time.append(data[5])
    GLL_lat.append(data[1] + data[2])
    GLL_lon.append(data[3] + data[4])
    GLL_valid.append(data[6])
    return ''

# $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
#     RMC      Recommended Minimum sentence C
#     123519       Fix taken at 12:35:19 UTC
#     A        Status A=active or V=Void.
#     4807.038,N   Latitude 48 deg 07.038' N
#     01131.000,E  Longitude 11 deg 31.000' E
#     022.4    Speed over the ground in knots
#     084.4    Track angle in degrees True
#     230394       Date - 23rd of March 1994
#     003.1,W      Magnetic Variation
def ProcRMC(data):
    RMC_time.append(data[1])
    RMC_lat.append(data[3] + data[4])
    RMC_lon.append(data[5] + data[6])
    RMC_valid.append(data[2])
    RMC_dir.append(data[8])
    RMC_speed.append(data[7])

NMEA_com = {
    '$GPVTG':ProcVTG,
    '$GPGGA':ProcGGA,
    '$GPGSA':ProcGSA,
    '$GPGSV':ProcGSV,
    '$GPGLL':ProcGLL,
    '$GPRMC':ProcRMC,
}

def WriteProcData(basefilename):
    print('Processing {}'.format(basefilename))
    
    fout = open(basefilename + '_GGA.csv', 'w')
    fout.write('Time,lat,lon,quality,num_sat,dop,zone,lat(m),lon(m)\n')
    for i in range(len(GGA_time)):
        zone, easting, northing = GeogToUTM(GGA_lat[i], GGA_lon[i])
        zone_c, easting_c, northing_c = LatLonToUTM(GGA_lat[i], GGA_lon[i])
#        print( ' * ' + GGA_lat[i]+' '+GGA_lon[i])
#        print('    '+str(zone)+' '+str(easting)+' '+str(northing))
#        print('    '+str(zone_c)+' '+str(easting_c)+' '+str(northing_c))
        fout.write(GGA_time[i]+','+GGA_lat[i]+','+GGA_lon[i]+','+GGA_quality[i]+','+GGA_num[i]+','+GGA_dop[i]+','+str(zone_c)+','+str(easting_c)+','+str(northing_c)+'\n')
    fout.close()
    
    fout = open(basefilename + '_GLL.csv', 'w')
    fout.write('Time,lat,lon,valid\n')
    for i in range(len(GLL_time)):
        fout.write(GLL_time[i]+','+GLL_lat[i]+','+GLL_lon[i]+','+GLL_valid[i]+'\n')
    fout.close()
    
    fout = open(basefilename + '_RMC.csv', 'w')
    fout.write('Time,lat,lon,valid,dir,speed\n')
    for i in range(len(RMC_time)):
        fout.write(RMC_time[i]+','+RMC_lat[i]+','+RMC_lon[i]+','+RMC_valid[i]+','+RMC_dir[i]+','+RMC_speed[i]+'\n')
    fout.close()

for f in FILES:
    VTG_dir = []
    VTG_speed = []
    GGA_time = []
    GGA_lat = []
    GGA_lon = []
    GGA_quality = []
    GGA_num = []
    GGA_dop = []
    GLL_time = []
    GLL_lat = []
    GLL_lon = []
    GLL_valid = []
    RMC_time = []
    RMC_lat = []
    RMC_lon = []
    RMC_valid = []
    RMC_dir = []
    RMC_speed = []

    fin = open(f, 'r')
    for line in fin:
        line = line.rstrip('\n')
        if (line[0] == '$'):
            data = line.split('*')
            data = data[0].split(',')
            try:
                NMEA_com[data[0][:6]](data)
            except:
                print('Command not found: {}'.format(line))
    fin.close()
    WriteProcData(f[:-4])

