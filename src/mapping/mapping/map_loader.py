import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String

from copy import deepcopy
from cv_bridge import CvBridge
from os import path
import cv2
import geojson
import pymap3d as pm
import numpy as np
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
import skimage as ski


class MapLoader(Node):
    """
    This node loads and publishes three maps:
    - Satellite imagery
    - Height
    - Planting bounds (generated by the user)
    """

    def __init__(self):
        super().__init__("map_loader")

        self.heightmap_pub = self.create_publisher(OccupancyGrid, "/map/height", 10)
        self.bounds_pub = self.create_publisher(OccupancyGrid, "/map/bounds", 10)
        self.imagery_pub = self.create_publisher(Image, "/map/imagery", 10)

        self.create_subscription(
            String, "/planning/bounds_geojson", self.processGeoJSON, 10
        )

        self.bridge = CvBridge()

        self.declareParams()

        json = """
{
    "type": "MultiPolygon",
    "coordinates": [
      [
        [
          [
            -79.94714534531758,
            40.44073342057788
          ],
          [
            -79.9471437936005,
            40.440709383025286
          ],
          [
            -79.94713915362578,
            40.44068557697594
          ],
          [
            -79.9471314700821,
            40.440662231694745
          ],
          [
            -79.9471208169691,
            40.440639572008905
          ],
          [
            -79.94710729688464,
            40.440617816142804
          ],
          [
            -79.94709104003664,
            40.44059717361634
          ],
          [
            -79.94707220298899,
            40.44057784322721
          ],
          [
            -79.94705096715373,
            40.44056001113644
          ],
          [
            -79.94702753704388,
            40.440543849075524
          ],
          [
            -79.94700213830386,
            40.440529512692706
          ],
          [
            -79.9469750155365,
            40.44051714005399
          ],
          [
            -79.94694642994729,
            40.44050685031352
          ],
          [
            -79.94691665682898,
            40.44049874256614
          ],
          [
            -79.94688598291039,
            40.44049289489311
          ],
          [
            -79.94687961142351,
            40.44049217558314
          ],
          [
            -79.94687374147337,
            40.44049057709157
          ],
          [
            -79.9468430675585,
            40.440484729418536
          ],
          [
            -79.94681178824715,
            40.44048119813554
          ],
          [
            -79.94680085979964,
            40.44048078952813
          ],
          [
            -79.94677869453452,
            40.44047656394298
          ],
          [
            -79.94675023622395,
            40.44047335113695
          ],
          [
            -79.94672353773446,
            40.44046608066192
          ],
          [
            -79.94669286383078,
            40.44046023298888
          ],
          [
            -79.9466615845308,
            40.44045670170589
          ],
          [
            -79.94663862615023,
            40.440455843306964
          ],
          [
            -79.94661624935114,
            40.440449749703845
          ],
          [
            -79.9465855754549,
            40.44044390203081
          ],
          [
            -79.94655711713659,
            40.440440689222335
          ],
          [
            -79.94653041863992,
            40.44043341874183
          ],
          [
            -79.94649974475115,
            40.44042757106879
          ],
          [
            -79.94647416411289,
            40.44042468313597
          ],
          [
            -79.94645290332379,
            40.44041703003871
          ],
          [
            -79.94644927515314,
            40.44041604202227
          ],
          [
            -79.94646604559495,
            40.440412844878956
          ],
          [
            -79.94649581864547,
            40.44040473707595
          ],
          [
            -79.94652440414477,
            40.4403944472677
          ],
          [
            -79.94655152679674,
            40.44038207455164
          ],
          [
            -79.94657692539386,
            40.44036773808492
          ],
          [
            -79.94660035533278,
            40.44035157593673
          ],
          [
            -79.94662159097007,
            40.440333743758686
          ],
          [
            -79.94664042779523,
            40.44031441328565
          ],
          [
            -79.94665668440034,
            40.440293770681855
          ],
          [
            -79.94667020422693,
            40.44027201474797
          ],
          [
            -79.94668085707383,
            40.440249355006515
          ],
          [
            -79.94668854035082,
            40.44022600968397
          ],
          [
            -79.94669318006673,
            40.44020220360917
          ],
          [
            -79.94669473154165,
            40.440178166047986
          ],
          [
            -79.94669446798513,
            40.440174083276496
          ],
          [
            -79.94669473150252,
            40.440170000505994
          ],
          [
            -79.94669317979844,
            40.440145962953395
          ],
          [
            -79.94668853986263,
            40.44012215690404
          ],
          [
            -79.94668085638334,
            40.440098811622846
          ],
          [
            -79.94667020335962,
            40.440076151937006
          ],
          [
            -79.94665668338847,
            40.44005439607091
          ],
          [
            -79.94664042667671,
            40.44003375354444
          ],
          [
            -79.94662158978694,
            40.440014423155326
          ],
          [
            -79.94660035412966,
            40.439996591064535
          ],
          [
            -79.94657692421616,
            40.439980429003626
          ],
          [
            -79.94655152568902,
            40.439966092620814
          ],
          [
            -79.94652440314898,
            40.43995371998208
          ],
          [
            -79.94649581779933,
            40.43994343024161
          ],
          [
            -79.94646604493055,
            40.439935322494236
          ],
          [
            -79.94643537126903,
            40.439929474821206
          ],
          [
            -79.94640885823868,
            40.43992648160356
          ],
          [
            -79.94640660517312,
            40.4399260520757
          ],
          [
            -79.94639565704247,
            40.43992105780412
          ],
          [
            -79.94636707170672,
            40.43991076806364
          ],
          [
            -79.9463372988524,
            40.43990266031627
          ],
          [
            -79.94630662520578,
            40.439896812643234
          ],
          [
            -79.94627534616797,
            40.439893281360234
          ],
          [
            -79.94624376296998,
            40.439892100475085
          ],
          [
            -79.94623406829987,
            40.43989246295554
          ],
          [
            -79.94621686795695,
            40.43988627141976
          ],
          [
            -79.94618709511347,
            40.43987816367238
          ],
          [
            -79.94615642147804,
            40.43987231599935
          ],
          [
            -79.94612514245162,
            40.439868784716346
          ],
          [
            -79.94610424710655,
            40.43986800344614
          ],
          [
            -79.94609524950366,
            40.4398638989545
          ],
          [
            -79.94606666419222,
            40.439853609214026
          ],
          [
            -79.94603689136319,
            40.43984550146665
          ],
          [
            -79.94600621774266,
            40.43983965379362
          ],
          [
            -79.94597493873145,
            40.43983612251062
          ],
          [
            -79.94595404335489,
            40.439835341238854
          ],
          [
            -79.94594504572501,
            40.43983123673292
          ],
          [
            -79.94591646042747,
            40.43982094699244
          ],
          [
            -79.94588668761291,
            40.439812839245064
          ],
          [
            -79.94585601400729,
            40.439806991572034
          ],
          [
            -79.94582473501126,
            40.43980346028904
          ],
          [
            -79.94579318207964,
            40.43980228053396
          ],
          [
            -79.94577698551376,
            40.439796450315825
          ],
          [
            -79.94574721271006,
            40.439788342568455
          ],
          [
            -79.94571653911561,
            40.439782494895425
          ],
          [
            -79.94568526013099,
            40.43977896361242
          ],
          [
            -79.9456536769867,
            40.439777782727276
          ],
          [
            -79.94564064920952,
            40.43977826983231
          ],
          [
            -79.94561997958344,
            40.43977432933457
          ],
          [
            -79.94558870060261,
            40.43977079805157
          ],
          [
            -79.94555711746217,
            40.43976961716642
          ],
          [
            -79.94552553432175,
            40.43977079805157
          ],
          [
            -79.9455088376999,
            40.43977268303936
          ],
          [
            -79.94549214107808,
            40.43977079805157
          ],
          [
            -79.94546055793764,
            40.43976961716642
          ],
          [
            -79.94542897479721,
            40.43977079805157
          ],
          [
            -79.94539769581637,
            40.43977432933457
          ],
          [
            -79.94537878536806,
            40.43977793445924
          ],
          [
            -79.94537472724915,
            40.439777782727276
          ],
          [
            -79.94534314410487,
            40.43977896361242
          ],
          [
            -79.94531186512025,
            40.439782494895425
          ],
          [
            -79.94529231726489,
            40.43978622153616
          ],
          [
            -79.94526804224864,
            40.43978712917228
          ],
          [
            -79.94526743888856,
            40.43978719728942
          ],
          [
            -79.94526683552849,
            40.43978712917228
          ],
          [
            -79.94523525238039,
            40.43978594828714
          ],
          [
            -79.94520366923228,
            40.43978712917228
          ],
          [
            -79.94517239024385,
            40.43979066045529
          ],
          [
            -79.94514171664567,
            40.43979650812832
          ],
          [
            -79.94511686976458,
            40.43980327444474
          ],
          [
            -79.94510953012613,
            40.439804673687185
          ],
          [
            -79.9450797573152,
            40.43981278143456
          ],
          [
            -79.94505117202111,
            40.43982307117503
          ],
          [
            -79.94502404953379,
            40.43983544381376
          ],
          [
            -79.944998651056,
            40.43984978019657
          ],
          [
            -79.94497522118805,
            40.439865942257484
          ],
          [
            -79.94497012029201,
            40.43987022561055
          ],
          [
            -79.94496449232429,
            40.43987410781536
          ],
          [
            -79.94494325670571,
            40.439891939906154
          ],
          [
            -79.94492441985024,
            40.43991127029528
          ],
          [
            -79.94490816316811,
            40.43993191282174
          ],
          [
            -79.94489464322159,
            40.43995366868784
          ],
          [
            -79.94488399021729,
            40.43997632837368
          ],
          [
            -79.94487630675198,
            40.43999967365488
          ],
          [
            -79.94487166682462,
            40.44002347970423
          ],
          [
            -79.94487011512338,
            40.44004751725682
          ],
          [
            -79.94487037864076,
            40.440051600034785
          ],
          [
            -79.94487011508424,
            40.44005568281369
          ],
          [
            -79.94487037860164,
            40.44005976559119
          ],
          [
            -79.9448701150451,
            40.44006384836959
          ],
          [
            -79.9448703785625,
            40.44006793114655
          ],
          [
            -79.94487011500597,
            40.44007201392449
          ],
          [
            -79.94487166647845,
            40.44009605148567
          ],
          [
            -79.94487630618703,
            40.440119857560475
          ],
          [
            -79.94488398945188,
            40.440143202883014
          ],
          [
            -79.94489464228195,
            40.44016586262447
          ],
          [
            -79.94490816208722,
            40.440187618558355
          ],
          [
            -79.94490821475704,
            40.440187685438495
          ],
          [
            -79.94489743319302,
            40.44020137567533
          ],
          [
            -79.94488391319231,
            40.44022313154144
          ],
          [
            -79.9448732601453,
            40.44024579122727
          ],
          [
            -79.9448655766492,
            40.44026913650847
          ],
          [
            -79.94486093670324,
            40.44029294255782
          ],
          [
            -79.94485938499578,
            40.4403169801104
          ],
          [
            -79.94486093647392,
            40.440341017671585
          ],
          [
            -79.94486557619938,
            40.44036482374639
          ],
          [
            -79.94487325949224,
            40.44038816906893
          ],
          [
            -79.94488391236115,
            40.44041082881039
          ],
          [
            -79.94489743221565,
            40.44043258474428
          ],
          [
            -79.94491368885433,
            40.44045322734808
          ],
          [
            -79.9449325257184,
            40.44047255782112
          ],
          [
            -79.94495376139953,
            40.44049038999916
          ],
          [
            -79.94497719138684,
            40.44050655214733
          ],
          [
            -79.9450025900364,
            40.44052088861406
          ],
          [
            -79.94502971274437,
            40.440533261330124
          ],
          [
            -79.94505829830268,
            40.44054355113838
          ],
          [
            -79.9450822705873,
            40.44055007926219
          ],
          [
            -79.94508842063789,
            40.440553550704735
          ],
          [
            -79.94511554335904,
            40.440565923420785
          ],
          [
            -79.94513241195705,
            40.44057199552729
          ],
          [
            -79.94513700101271,
            40.44057408894099
          ],
          [
            -79.94515386961054,
            40.440580161046704
          ],
          [
            -79.94515845866637,
            40.44058225446022
          ],
          [
            -79.94518704424553,
            40.440592544268476
          ],
          [
            -79.94521681737923,
            40.44060065207148
          ],
          [
            -79.94522415701466,
            40.440602051307174
          ],
          [
            -79.94524900387606,
            40.440608817589656
          ],
          [
            -79.94525634351106,
            40.4406102168251
          ],
          [
            -79.94527538948728,
            40.440615403413354
          ],
          [
            -79.945281539513,
            40.4406188748385
          ],
          [
            -79.94530866226052,
            40.440631247554556
          ],
          [
            -79.9453372478605,
            40.44064153736281
          ],
          [
            -79.94536702101591,
            40.44064964516581
          ],
          [
            -79.94539769499241,
            40.44065549288018
          ],
          [
            -79.94540764424468,
            40.440656616108015
          ],
          [
            -79.9454338073401,
            40.440666033896605
          ],
          [
            -79.94546358050637,
            40.44067414169961
          ],
          [
            -79.94549425449404,
            40.44067998941397
          ],
          [
            -79.94550420373481,
            40.44068111264011
          ],
          [
            -79.94553036681971,
            40.44069053042147
          ],
          [
            -79.9455601399968,
            40.44069863822447
          ],
          [
            -79.94557204715049,
            40.44070090821326
          ],
          [
            -79.94559834064961,
            40.44071290263248
          ],
          [
            -79.9456269262843,
            40.44072319244074
          ],
          [
            -79.94565669947588,
            40.44073130024373
          ],
          [
            -79.94566249466877,
            40.44073240504316
          ],
          [
            -79.94567344244669,
            40.44073739913654
          ],
          [
            -79.94570202809183,
            40.44074768894479
          ],
          [
            -79.94573180129423,
            40.44075579674779
          ],
          [
            -79.94574370842165,
            40.44075806672964
          ],
          [
            -79.94575984265018,
            40.44076542674419
          ],
          [
            -79.94577506555859,
            40.440774019401765
          ],
          [
            -79.9458021883687,
            40.44078639211782
          ],
          [
            -79.94583077403466,
            40.44079668192608
          ],
          [
            -79.94586054725877,
            40.440804789729086
          ],
          [
            -79.94589122130606,
            40.44081063744345
          ],
          [
            -79.94592250076575,
            40.4408141687519
          ],
          [
            -79.94593342913471,
            40.440814577357315
          ],
          [
            -79.9459555943148,
            40.440818802936846
          ],
          [
            -79.9459619657245,
            40.440819522239885
          ],
          [
            -79.94596783559666,
            40.440821120714915
          ],
          [
            -79.9459985096514,
            40.440826968429285
          ],
          [
            -79.94602696787314,
            40.44083018123205
          ],
          [
            -79.9460536662624,
            40.44083745169677
          ],
          [
            -79.94608434032459,
            40.44084329941114
          ],
          [
            -79.94609607649245,
            40.44084462437037
          ],
          [
            -79.94609899484628,
            40.440845674871674
          ],
          [
            -79.94612876809211,
            40.44085378267467
          ],
          [
            -79.94615944216174,
            40.440859630389035
          ],
          [
            -79.94618502259004,
            40.44086251830072
          ],
          [
            -79.94620628316194,
            40.44087017133106
          ],
          [
            -79.94623605641861,
            40.44087827913406
          ],
          [
            -79.94626673049942,
            40.44088412684843
          ],
          [
            -79.94629231091619,
            40.440887014757756
          ],
          [
            -79.9463135714776,
            40.44089466778155
          ],
          [
            -79.94634334474512,
            40.44090277558455
          ],
          [
            -79.94637401883712,
            40.44090862329892
          ],
          [
            -79.94640247703978,
            40.44091183609563
          ],
          [
            -79.94642917541088,
            40.44091910654657
          ],
          [
            -79.94645984951033,
            40.440924954260936
          ],
          [
            -79.94649112902322,
            40.440928485569394
          ],
          [
            -79.9465140871791,
            40.44092934396015
          ],
          [
            -79.94653646374876,
            40.44093543750464
          ],
          [
            -79.94656713785565,
            40.44094128521901
          ],
          [
            -79.94659559605054,
            40.44094449801328
          ],
          [
            -79.9466222944145,
            40.44095176845874
          ],
          [
            -79.94665296852885,
            40.44095761617311
          ],
          [
            -79.94668424805693,
            40.44096114748156
          ],
          [
            -79.94669517639117,
            40.44096155608479
          ],
          [
            -79.94671734153758,
            40.44096578164866
          ],
          [
            -79.946723712913,
            40.440966500946246
          ],
          [
            -79.94672958275238,
            40.44096809940886
          ],
          [
            -79.94676025687419,
            40.440973947123226
          ],
          [
            -79.94679153640986,
            40.440977478431684
          ],
          [
            -79.94682312011722,
            40.44097865932544
          ],
          [
            -79.94685470382456,
            40.440977478431684
          ],
          [
            -79.94688598336023,
            40.440973947123226
          ],
          [
            -79.94691665748203,
            40.44096809940886
          ],
          [
            -79.94694643077848,
            40.44095999160586
          ],
          [
            -79.94697501651387,
            40.4409497017976
          ],
          [
            -79.94700213938987,
            40.44093732908155
          ],
          [
            -79.94702753819676,
            40.440922992614816
          ],
          [
            -79.94705096832921,
            40.44090683046664
          ],
          [
            -79.94707220414188,
            40.440888998288585
          ],
          [
            -79.94709104112265,
            40.44086966781555
          ],
          [
            -79.947107297862,
            40.44084902521175
          ],
          [
            -79.9471208178003,
            40.44082726927786
          ],
          [
            -79.94713147073516,
            40.4408046095364
          ],
          [
            -79.94713915407563,
            40.44078126421386
          ],
          [
            -79.94714379382984,
            40.44075745813907
          ],
          [
            -79.94714534531758,
            40.44073342057788
          ]
        ]
      ]
    ]
  }

"""

        # self.bounds_img = self.processGeoJSON(String(data=json))

        self.bounds_img = None

        # self.publishBoundsMsg()

        # self.heightmap_msg, self.imagery_msg, self.bounds_msg = self.loadMapData()
        self.bounds_msg = None

        self.create_timer(5, self.publishBoundsMsg)

    def publishBoundsMsg(self):
        if self.bounds_img is None:
            return

        bounds_grid_msg = OccupancyGrid()
        bounds_grid_msg.header = self.getHeader()
        bounds_grid_msg.info.height = self.bounds_img.shape[0]
        bounds_grid_msg.info.width = self.bounds_img.shape[1]
        bounds_grid_msg.info.resolution = self.get_parameter("resolution").value
        bounds_grid_msg.info.map_load_time = self.get_clock().now().to_msg()
        bounds_grid_msg: OccupancyGrid = deepcopy(bounds_grid_msg)

        bounds_grid_msg.data = self.bounds_img.astype(int).flatten().tolist()
        self.bounds_pub.publish(bounds_grid_msg)

    def publishMaps(self) -> None:
        if self.bounds_msg is None:
            return
        self.bounds_pub.publish(self.bounds_msg)
        # self.heightmap_pub.publish(self.heightmap_msg)
        # self.imagery_pub.publish(self.imagery_msg)
        self.get_logger().debug("Publishing maps!")

    def processGeoJSON(self, json_msg: String, lat0=40.44019, lon0=-79.94719):
        res = self.get_parameter("resolution").value

        try:
            json = geojson.loads(json_msg.data)
            h = 280.3  # meters
        except Exception as e:
            self.get_logger().error(f"Message '{json_msg.data} was not valid. {e}")
            return

        point_arr = []
        for point in list(geojson.utils.coords(json)):
            x, y, z = pm.geodetic2enu(point[1], point[0], h, lat0, lon0, h)
            # print(x, y, z)
            point_arr.append([x, y])

        # An array of points in ENU coordinates,
        # relative to lat0, lon0.
        point_arr = np.asarray(point_arr)
        # plt.scatter(point_arr[:, 0], point_arr[:, 1])
        # plt.show()

        # Convert into pixel space by applying resolution...
        point_arr /= res

        # ... and translating such that the bottom point has y=0
        # and the left point has x=0...

        min_x = np.min(point_arr[:, 0])
        min_y = np.min(point_arr[:, 1])
        point_arr[:, 0] -= min_x
        point_arr[:, 1] -= min_y

        # ...and rounding to int.
        point_arr = point_arr.astype(int)
        # plt.scatter(point_arr[:, 0], point_arr[:, 1])
        # plt.show()

        width = np.max(point_arr[:, 0]) + 1
        height = np.max(point_arr[:, 1]) + 1

        img = np.ones((height, width)) * 100  # 100 = "Don't plant here"
        rr, cc = ski.draw.polygon(point_arr[:, 1], point_arr[:, 0])
        img[rr, cc] = 0

        self.get_logger().info(f"Calculated bounds of shape {img.shape}")

        self.bounds_img = img

    def declareParams(self) -> None:
        # Declare the map name as a ROS parameter
        descr = ParameterDescriptor()
        descr.description = "The directory containing bounds.npy, imagery.jpg, heightmap.jpg, and info.yaml"
        descr.type = ParameterType.PARAMETER_STRING
        self.declare_parameter(
            "map_dir",
            descriptor=descr,
            value="/home/main/arbor/steward/data/maps/flagstaff/",
        )

        # Map resolution
        descr = ParameterDescriptor()
        descr.description = "Map resolution (m/pixel)"
        descr.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter("resolution", descriptor=descr, value=0.2)

        # Map origin
        descr = ParameterDescriptor()
        descr.description = "Map origin [x,y,z]"
        descr.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter("origin", descriptor=descr)

    def getHeader(self) -> Header:
        """Forms a message Header.

        Returns:
            Header: The stamped and framed Header.
        """
        header = Header()

        # Our map is in the... map frame. Yes, really!
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg()

        return header

    def loadHeightMap(self, dir: str) -> np.ndarray:
        heightmap = cv2.imread(path.join(dir, "heightmap.jpg"))

        # For some reason, the data shows up flipped along the vertical
        # axis in Rviz. Don't ask why! WSH.
        heightmap = cv2.flip(heightmap, 0)

        # Convert to grayscale from RGB, RGBA etc if necessary
        if heightmap.ndim > 2:
            heightmap = cv2.cvtColor(heightmap, cv2.COLOR_BGR2GRAY)

        # Convert from (0,255) to (0,100)
        heightmap = (heightmap.astype(float) * (100 / 255)).astype(int)

        return heightmap

    def loadImagery(self, dir: str) -> Image:
        imagery = cv2.imread(path.join(dir, "imagery.jpg"))

        image_msg = self.bridge.cv2_to_imgmsg(imagery, encoding="bgr8")
        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        return image_msg

    def loadBounds(self, dir: str) -> np.ndarray:
        """Load bounds from a .npy file-- not a .jpg!
        We assume that the bounds have already been converted to a .npy file.
        Support may be added for other methods in the future.

        Args:
            dir (str): Directory where bounds.npy is stored

        Returns:
            np.ndarray: Bounds, as a numpy array
        """
        try:
            bounds = np.load(path.join(dir, "bounds.npy"))
            # For some reason, the data shows up flipped along the vertical
            # axis in Rviz. Don't ask why! WSH.
            bounds = np.flip(bounds, axis=0)
            return bounds
        except Exception as e:
            # If the bounds couldn't be loaded, return a blank 4 acre x 4 acre image
            self.get_logger().error(
                f"bounds.npy could not be loaded! Returning blank map. {e}"
            )
            return np.zeros((1272, 1272))

    def loadMapData(self) -> tuple[np.ndarray, Image, np.ndarray]:

        # Actually load the value of the ROS param
        map_dir = self.get_parameter("map_dir").value
        heightmap = self.loadHeightMap(map_dir)
        bounds = self.loadBounds(map_dir)
        imagery = self.loadImagery(map_dir)

        heightmap_grid_msg = OccupancyGrid()
        heightmap_grid_msg.header = self.getHeader()
        heightmap_grid_msg.info.height = heightmap.shape[0]
        heightmap_grid_msg.info.width = heightmap.shape[1]
        heightmap_grid_msg.info.resolution = self.get_parameter("resolution").value
        heightmap_grid_msg.info.map_load_time = self.get_clock().now().to_msg()

        origin = self.get_parameter("origin").value
        heightmap_grid_msg.info.origin.position.x = origin[0]
        heightmap_grid_msg.info.origin.position.y = origin[1]
        heightmap_grid_msg.info.origin.position.z = origin[2]

        bounds_grid_msg: OccupancyGrid = deepcopy(heightmap_grid_msg)

        heightmap_grid_msg.data = heightmap.flatten().tolist()
        bounds_grid_msg.data = bounds.flatten().tolist()

        # plt.imshow(heightmap)
        # plt.imshow(bounds)
        # plt.show()

        return heightmap_grid_msg, imagery, bounds_grid_msg


def main(args=None):
    rclpy.init(args=args)

    node = MapLoader()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
