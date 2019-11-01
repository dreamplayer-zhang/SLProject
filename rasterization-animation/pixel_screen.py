from manimlib.imports import *

# not a scene!
class PixelBox(Scene):
    # pass kwargs json and vertices json as parameter
    def __init__(self, kwargs, vertices):
        self.screen_pixels = kwargs
        self.boxes, self.height, self.width = self.getPixelBoxes()
        self.vertex_box = self.get_vertices(vertices)
        self.border_lines = self.get_border_lines()
        self.border_boxes = self.get_border_boxes(vertices)

    # returns all boxes(pixels), numbers on the horizontal side, numbers on the vertical side
    def getPixelBoxes(self):
        pixels = []
        numbers_height = []
        numbers_width = []
        height = 0
        width = 0


        start = self.screen_pixels["startpoint"].copy()
        pixel_point = self.screen_pixels["startpoint"]
        side_length = self.screen_pixels["res_width"] * float(self.screen_pixels["side_length"])
        side_length2 = side_length * 2

        numbers_height.append(self.generatePositionText(pixel_point, height))
        height += 1

        for pixel in range(int(self.screen_pixels["amount"])):
            nPixel = Square(**self.screen_pixels)

            if pixel_point[0] > side_length:
                pixel_point[1] += 0.5
                pixel_point[0] = 0.5
                numbers_height.append(self.generatePositionText(pixel_point, height))
                height += 1

            if ((side_length2) > pixel):
                numbers_width.append(self.generatePositionText(pixel_point, width, DOWN))
                width += 1

            nPixel.move_to(pixel_point)

            pixels.append(nPixel)
            pixel_point[0] += 0.5

        self.screen_pixels["startpoint"] = start
        return pixels, numbers_height, numbers_width

    def generatePositionText(self, pos, number, direction=LEFT):
        return TexMobject(number).next_to(pos, direction).scale(0.5)

    # colors in the vertex and returns it as a list of squares
    def get_vertices(self, vertices):
        vertex_box = []
        for vertex in vertices:
            newPixel = Square(**self.screen_pixels) \
                .move_to(self.get_box(vertex["position"][0], vertex["position"][1]).get_center())
            newPixel.set_color(vertex["color"]).set_fill(vertex["color"], 1)
            vertex_box.append(newPixel)
        return vertex_box

    # returns lines between 3 vertecies
    def get_border_lines(self, stroke=4):

        line1 = Line(self.vertex_box[0].get_center(),
                     self.vertex_box[1].get_center(),
                     stroke_width=stroke)

        line2 = Line(self.vertex_box[1].get_center(),
                     self.vertex_box[2].get_center(),
                     stroke_width=stroke)

        line3 = Line(self.vertex_box[2].get_center(),
                     self.vertex_box[0].get_center(),
                     stroke_width=stroke)

        return line1, line2, line3

    # bresenham draw line
    def get_border_boxes(self, vertices):

        line1 = self.bresenham(vertices[0]["position"][0], vertices[0]["position"][1],
                               vertices[1]["position"][0], vertices[1]["position"][1])

        line2 = self.bresenham(vertices[1]["position"][0], vertices[1]["position"][1],
                               vertices[2]["position"][0], vertices[2]["position"][1])

        line3 = self.bresenham(vertices[2]["position"][0], vertices[2]["position"][1],
                               vertices[0]["position"][0], vertices[0]["position"][1])

        return line1, line2, line3

    # returns list of boxes that are a line from point1(x0, y0) to point2(x1, y1) with bresenham
    def bresenham(self, x0, y0, x1, y1):
        b_boxes = []

        dx = x1 - x0
        dy = y1 - y0
        stepx, stepy = 0, 0
        if dy < 0:
            dy = -dy
            stepy = -1
        else:
            stepy = 1
        if dx < 0:
            dx = -dx
            stepx = -1
        else:
            stepx = 1
        if dx > dy:
            b_boxes = self.bresenhamloop(dx, dy, x0, x1, y0, y1, stepy, stepx, False)
        else:
            # swap all values with exception of stepy and stepx
            # TODO fix bresenham other loop!
            b_boxes = self.bresenhamloop(dy, dx, y0, y1, x0, x1, stepx, stepy, True)
        return b_boxes

    def bresenhamloop(self, dx, dy, x0, x1, y0, y1, stepy, stepx, swap):
        b_boxes = []

        dE = 2 * dy
        dNE = 2 * dy - 2 * dx
        D = 2 * dy - dx
        b_boxes.append(self.get_box(y0, x0)) if swap else b_boxes.append(self.get_box(x0, y0))

        while x0 != x1:
            if D < 0:
                D += dE
            else:
                D += dNE
                y0 += stepy
            x0 += stepx
            b_boxes.append(self.get_box(y0, x0)) if swap else b_boxes.append(self.get_box(x0, y0))

        return b_boxes

    # gets square from its x and y position
    def get_box(self, x, y):
        index = x + (self.screen_pixels["res_width"] * y)
        return self.boxes[index]

    # gets all Squares (as list) on this y cordinate (height)
    def get_box_line_x(self, y):
        x_line = []
        for i in range(self.screen_pixels["res_width"]):
            x_line.append(self.get_box(i, y))
        return x_line

    # returns x and y index of the square (box)
    def get_position(self, search_box):

        for i in range(len(self.boxes)):
            if self.boxes[i].get_x() == search_box.get_x() and self.boxes[i].get_y() == search_box.get_y():
                y = int(i / self.screen_pixels["res_width"])
                x = int(i - (y * self.screen_pixels["res_width"]))
                return x, y
        else:
            return None

    def get_line_between_boxes(self, x0, x1, y):
        if x1 - x0 < 1:
            return None
        n = x1 - x0
        line_between = []
        for i in range(1, n):
            line_between.append(self.get_box(x0 + i, y))
        return line_between


# one dimensional Line
class PixelLine(Scene):

    def __init__(self, kwargs):
        self.line_pixels = kwargs
        self.boxes = self.getPixelBoxes()

    def getPixelBoxes(self):
        pixels = []

        firstSquare = Square(**self.line_pixels)
        firstSquare.set_color(BLUE).set_fill(BLUE, 1)
        firstSquare.to_edge(DOWN).shift(LEFT * 4.5 + UP * 0.5)

        start_point = firstSquare

        pixels.append(firstSquare)

        for i in range(1, self.line_pixels["amount"]):
            new_pixel = Square(**self.line_pixels)
            new_pixel.next_to(start_point)
            start_point = new_pixel
            pixels.append(new_pixel)
        pixels[len(pixels) - 1].set_color(RED).set_fill(RED, 1)
        return pixels


class InterpolatedPoly(Scene):
    def construct(self):



        red_dot = Dot(np.array((1,-1,0)))
        red_dot.set_color(RED)

        blue_dot = Dot(np.array((0,1,0)))
        blue_dot.set_color(BLUE)

        green_dot = Dot(np.array((-1,-1,0)))
        green_dot.set_color(GREEN)

        self.play(AnimationGroup(FadeIn(red_dot),
                  FadeIn(blue_dot),
                  FadeIn(green_dot), lag_ratio=0.5))

        self.wait(3)

        image = ImageMobject("polygon")
        image.scale(1.3)
        self.play(FadeIn(image))

        self.wait(3)

        self.play(
            FadeOut(red_dot),
            FadeOut(blue_dot),
            FadeOut(green_dot),
            FadeOut(image)
        )

        self.wait()


class ScanLine(Scene):
    CONFIG = {
        "line_pixels": {
            "color": WHITE,
            "amount": 8,
            "side_length": 1,
            "stroke_width": 2,
            "stroke_opacity": 1
        },
        "screen_pixels": {
            "color": WHITE,
            "amount": 144,  # 12 x 12
            "side_length": 0.5,
            "res_width": 12,
            "res_height": 12,
            "startpoint": DOWN * 2.75 + RIGHT * 0.5,
            "stroke_opacity": 0.5,
            "stroke_width": 0.8,

        },
        "end_box": {
            "color": BLACK,
            "side_length": FRAME_WIDTH,
            "mark_paths_closed": True,
            "close_new_points": True,
            "center": ORIGIN,
        },
        "vertices": (
            {
                "index": 128,
                "color": RED,
                "fill_opacity": 0.0,
                "position": (8, 10),
                "color_rgb": (255, 0, 0),  # for vertex
            },
            {
                "index": 37,
                "color": BLUE,
                "fill_opacity": 0.0,
                "position": (1, 3),
                "color_rgb": (0, 200, 255),  # for vertex
            },
            {
                "index": 21,
                "color": GREEN,
                "fill_opacity": 0.0,
                "position": (9, 1),
                "color_rgb": (0, 255, 0),  # for vertex
            }

        ),
        "script": {
            "title": "Scanline algorithm",
        }
    }
    def construct(self):

        # interpolation start
        vertex_save = TexMobject("[", "v0", ",", "v1", ",", "v2", ",", "...", "]"
                                 ).to_corner(UR).scale(0.5)

        self.add(vertex_save)
        pixel = self.setup_scanline()


        title_scanline = TextMobject(self.script["title"]).to_corner(UL)

        self.play(Write(title_scanline), run_time=1.5)

        self.wait(4)

        sorted_list_tex, all_x_on_y = self.sort_scanline(pixel)

        #                               0        1     2    3     4    5    6
        draw_line_tex = TexMobject("drawLine(", "x1", ",", "x2", ",", "y", ")").to_edge(LEFT).shift(DOWN)

        self.play(sorted_list_tex.shift, UP)
        self.play(Write(draw_line_tex))
        self.wait(8)

        pointer = Arrow(LEFT,ORIGIN).move_to(pixel.get_box_line_x(10)[0]).shift(LEFT)
        pointer.set_stroke(WHITE,8).shift(LEFT)

        old_list = sorted_list_tex[3]
        old_index = sorted_list_tex[1].shift(UP * 0.05 + RIGHT*0.05)
        sorted_list_tex[2].shift(RIGHT * 0.10)

        old_focus_start = None
        old_focus_end = None

        for i in range(1, len(pixel.height)-1).__reversed__():
            line_x = pixel.get_box_line_x(i)
            n_list = self.generate_list_x_tex(all_x_on_y[i], sorted_list_tex[2])
            n_index = TexMobject(str(i)).move_to(sorted_list_tex[1])
            numbers = self.get_numbers_x(all_x_on_y, i, pixel)

            self.play(pointer.move_to, line_x[0], pointer.shift, LEFT,
                      ReplacementTransform(old_list, n_list),
                      ReplacementTransform(old_index, n_index),
                      *[FadeIn(number) for number in numbers])

            start_x_goal = self.get_start_x_index(all_x_on_y[i])
            end_x_goal = len(all_x_on_y[i]) - 1
            n_focus_start = SurroundingRectangle(pixel.get_box(all_x_on_y[i][start_x_goal], i), buff=0.01)
            n_focus_end = SurroundingRectangle(pixel.get_box(all_x_on_y[i][end_x_goal], i), buff=0.01)
            start_x_tex = n_list[start_x_goal+(start_x_goal+1)].copy()
            end_x_tex = n_list[end_x_goal + (end_x_goal+1)].copy()

            # start
            if i == len(pixel.height)-2:
                self.play(ShowCreation(n_focus_start),
                          ShowCreation(n_focus_end),
                          start_x_tex.set_color,YELLOW,
                          end_x_tex.set_color,YELLOW)
                self.wait(6)

            else:
                self.play(ReplacementTransform(old_focus_start, n_focus_start),
                          ReplacementTransform(old_focus_end, n_focus_end),
                          start_x_tex.set_color, YELLOW,
                          end_x_tex.set_color, YELLOW)

            self.wait()


            self.play(ReplacementTransform(start_x_tex, draw_line_tex[1]),
                      ReplacementTransform(end_x_tex, draw_line_tex[3]),
                      ReplacementTransform(n_index.copy(), draw_line_tex[5]),
                      *[FadeOut(number) for number in numbers])

            self.draw_line_box(all_x_on_y[i][start_x_goal], all_x_on_y[i][end_x_goal], i, pixel)

            old_list = n_list
            old_index = n_index
            old_focus_start = n_focus_start
            old_focus_end = n_focus_end


        self.wait()

        self.play(FadeOut(old_focus_start), FadeOut(old_focus_end),
                  FadeOutAndShiftDown(draw_line_tex),
                  FadeOutAndShiftDown(sorted_list_tex[0]),
                  FadeOutAndShiftDown(sorted_list_tex[2]),
                  FadeOutAndShiftDown(old_list),
                  FadeOutAndShiftDown(old_index),
                  FadeOut(pointer))

        self.wait()

        self.play(FadeOut(title_scanline))

        self.wait(5)

        self.play(FadeIn(Square(**self.end_box).set_fill(BLACK,1)))

    def draw_line_box(self,x1,x2,y, pixel):
        if x2 - x1 <= 1:
            return
        line = pixel.bresenham(x1, y, x2, y)
        speed = 3 / (3 * (len(line)-2))
        self.interpolate_line_ani(line,speed)

    def sort_scanline(self, pixel):
        sort_scanline_template = TexMobject(r"{ y }_{ n }=\begin{bmatrix} all\quad x\quad with\quad { y }_{ n } \end{bmatrix}")
        sort_scanline_template.to_edge(LEFT)

        all_x_on_y = self.get_all_x_on_y(pixel)

        sort_scanline_tex_group = []
        for number, i in zip(pixel.height, range(len(pixel.height))):
            temp0 = TexMobject("{ y }_{", number.get_tex_string(), "}= [", "\\quad]")

            temp0.move_to(number).to_edge(LEFT).scale(0.8)

            sort_scanline_tex_group.append(temp0)

        self.play(Write(sort_scanline_template))
        self.wait(5)
        self.play(FadeOut(sort_scanline_template))

        pointer = Arrow(LEFT,ORIGIN).shift(LEFT)
        pointer.set_stroke(WHITE,8).shift(LEFT)

        self.play(
            *[ReplacementTransform(number.copy(),  tex[1])
              for number, tex in zip(pixel.height, sort_scanline_tex_group)]
        )
        self.play(
            *[FadeIn(tex[i]) for i in (0, 2, 3) for tex in sort_scanline_tex_group]
        )
        uglyList = []
        for i in range(len(pixel.height)).__reversed__():
            line_x = pixel.get_box_line_x(i)
            self.play(pointer.move_to, line_x[0], pointer.shift, LEFT)
            self.play(*[FadeIn(box) for box in line_x], run_time=0.3)
            if len(all_x_on_y[i]) > 1:
                points = self.show_numbers_x(all_x_on_y, i, pixel, False)

                if len(all_x_on_y[i]) > 1:
                    temp1 = TexMobject(str(all_x_on_y[i]).strip('[]'), "]")
                else:
                    temp1 = TexMobject("\\quad", "]")
                temp1.scale(0.8).next_to(sort_scanline_tex_group[i][2]).shift(LEFT*0.2)
                uglyList.append(temp1)
                self.play(ReplacementTransform(sort_scanline_tex_group[i][3], temp1[1]),
                          *[ReplacementTransform(point, temp1[0]) for point in points])

            if(i < 8):
                speed = 0.5

        self.wait()
        all_x_on_y_tex = TexMobject("all\_ x\_ on\_ y= [", "...", "]").to_edge(LEFT*0.2)

        self.play(FadeOut(pointer),
                *[ReplacementTransform(tex, all_x_on_y_tex[1]) for tex in sort_scanline_tex_group],
                *[ReplacementTransform(tex, all_x_on_y_tex[1]) for tex in uglyList])

        self.wait()
        self.play(FadeIn(all_x_on_y_tex[0]), FadeIn(all_x_on_y_tex[2]))

        self.wait(4)
        all_x_on_y_tex_active = TexMobject("all\_ x\_ on\_ y[", "y", "]=", "[x1,...]").to_edge(LEFT)

        self.play(ReplacementTransform(all_x_on_y_tex, all_x_on_y_tex_active))

        self.wait(6)

        return all_x_on_y_tex_active, all_x_on_y

    def dev_sort(self, pixel):
        all_x_on_y = self.get_all_x_on_y(pixel)
        all_x_on_y_tex_active = TexMobject("all\_ x\_ on\_ y [", " y ", "] =", "[x1,...]").to_edge(LEFT)
        self.add(all_x_on_y_tex_active)

        return all_x_on_y_tex_active, all_x_on_y

    def generate_list_x_tex(self, x_list, pos):
        list_tex = []
        list_tex.append("[")
        for i in range(len(x_list)):
            if i == len(x_list)-1:
                list_tex.append(str(x_list[i]))
            else:
                list_tex.append(str(x_list[i]))
                list_tex.append(",")
        list_tex.append("]")

        return TexMobject(*list_tex).next_to(pos)

    def get_start_x_index(self, list_x):
        for i in range(len(list_x)):
            if i == len(list_x)-2:
                return i
            if list_x[i] + 1 < list_x[i+1]:
                return i

    def show_numbers_x(self,all_x_on_y,y,pixel,fade_out=True):
        if len(all_x_on_y[y]) > 1 or not fade_out:
            points = self.get_numbers_x(all_x_on_y, y, pixel)
            self.play(*[FadeIn(p) for p in points])
            if fade_out:
                self.wait()
                self.play(*[FadeOut(p) for p in points])
            return points

    def get_numbers_x(self,all_x_on_y,y,pixel):
            points = []
            for x in all_x_on_y[y]:
                points.append(TexMobject(str(x)).move_to(pixel.get_box(x, y).get_center()))
            return points

    def get_all_x_on_y(self,pixel):
        all_x_on_y = []
        for i in range(12):
            all_x_on_y.append([])

        for line in pixel.border_boxes:
            for box in line:
                x, y = pixel.get_position(box)
                all_x_on_y[y].append(x)

        return all_x_on_y


    def setup_scanline(self):


        pixel_line_1D = PixelLine(self.line_pixels)

        pixel_line_1D.screen_pixels = self.screen_pixels

        self.interpolate_line(pixel_line_1D.boxes)



        #scanline start
        pixel = PixelBox(self.screen_pixels, self.vertices)
        pixel_lines = pixel.border_boxes


        for vertex in self.vertices:
            actual_pixel = pixel.boxes[vertex["index"]]
            actual_pixel.set_color(vertex["color"]).set_fill(vertex["color"], 1)

        for pixel_line in pixel_lines:
            self.interpolate_line(pixel_line)


        self.add(*[box for box in pixel_line_1D.boxes])

        temp_border_boxes = [box.copy() for box in pixel.border_boxes[0]]
        pixel_line_1D.boxes.reverse()
        # vielleicht muss man die box_end noch rausfaden!
        self.play(*[ReplacementTransform(box_start, box_end)
                    for box_start, box_end in
                    zip(pixel_line_1D.boxes,
                        temp_border_boxes)], run_time=2)

        self.wait()

        self.play(
                *[FadeIn(number) for number in pixel.height],
                *[FadeIn(number) for number in pixel.width],
                *[FadeIn(box) for box in pixel.boxes]
                )

        self.remove(*[temp_box for temp_box in temp_border_boxes])

        self.wait()


        return pixel

    def interpolate_line_ani(self,line, time=0.5):
        per = 100 / (len(line)-1)
        for i in range(1, len(line)-1):
            new_color = self.interpolate_pixel(per * i / 100,
                                               line[0].get_color(),
                                               line[len(line)-1].get_color())
            self.play(line[i].set_fill, new_color, 1,
                      line[i].set_color, new_color, run_time=time)

    def interpolate_line(self,line):
        per = 100 / (len(line)-1)
        for i in range(1, len(line)-1):
            new_color = self.interpolate_pixel(per * i / 100,
                                               line[0].get_color().get_hex_l(),
                                               line[len(line)-1].get_color().get_hex_l())

            line[i].set_fill(new_color, 1)
            line[i].set_color(new_color)



    def interpolate_pixel(self, amount,start_color, end_color):
        rgb_inter = interpolate(hex_to_rgb(str(start_color)),
                                hex_to_rgb(str(end_color)), amount)
        return rgb_to_hex(rgb_inter)

class Interpolation(Scene):
    CONFIG = {
        "line_pixels": {
            "color": WHITE,
            "amount": 8,
            "side_length": 1,
            "stroke_width": 2,
            "stroke_opacity": 1
        },
        "script": {
            "save": "Save each color to its realtive position"
        }
    }


    def define_interpolation(self):
        vertex_save = TexMobject("[", "v0", ",", "v1", ",", "v2", ",", "...", "]"
                                 ).to_corner(UR).scale(0.5)

        pixel_line = PixelLine(self.line_pixels)
        boxes = pixel_line.boxes
        n = len(boxes) - 1

        interpolation_text = TextMobject("Interpolation").scale(1).to_corner(UL) \
            .set_color_by_gradient(BLUE_A, RED)

        self.add(vertex_save, interpolation_text, *[box for box in boxes])


        start_end = DoubleArrow(boxes[0].get_corner(UL) + UP * 0.5,
                                boxes[n].get_corner(UR) + UP * 0.5)

        length_text = TextMobject("length:" + str(len(boxes))) \
            .next_to(start_end.get_center(), UP) \
            .scale(0.5)

        blue_amount = DecimalNumber(100,
                                    num_decimal_places=0,
                                    unit="\\rm \\%").set_color(BLUE)

        red_amount = DecimalNumber(100,
                                   num_decimal_places=0,
                                   unit="\\rm \\%").set_color(RED)

        blue_amount.next_to(boxes[0].get_center(), UP * 6)
        red_amount.next_to(boxes[n].get_center(), UP * 6)

        focus_rect = SurroundingRectangle(boxes[0], buff=0.15)

        red_matrix = self.show_rgb_vector(RED).get_tex_string()
        blue_matrix = self.show_rgb_vector(BLUE).get_tex_string()


        linear_interpolation_tex = TexMobject("\\frac{"+str(n)+"}{" + str(n) + "}",     #0
                                              "* ",                                         #1
                                              blue_matrix,                                  #2
                                              "+",                                          #3
                                              "\\frac{" + str(0) + "}{" + str(n) + "}", #4
                                              "* ",                                         #5
                                              red_matrix,                                   #6
                                              "=",                                          #7
                                              blue_matrix)                                  #8
        linear_interpolation_title = TextMobject("Linear Interpolation")\
            .next_to(linear_interpolation_tex, UP*4.5)\
            .scale(0.7)

        linear_interpolation_tex.shift(UP)
        linear_interpolation_tex[2].set_color(BLUE)
        linear_interpolation_tex[6].set_color(RED)
        linear_interpolation_tex[8].set_color(BLUE)


        self.play(
            FadeInFromDown(blue_amount),
            FadeInFromDown(red_amount)
        )
        self.play(
            GrowFromCenter(start_end),
            Write(length_text)
        )

        self.wait(2)

        self.play(FadeOutAndShiftDown(start_end),
                  FadeOutAndShiftDown(length_text),
                  blue_amount.next_to, focus_rect, UP,
                  red_amount.next_to, focus_rect, UP * 3)

        self.play(ChangeDecimalToValue(red_amount, 0), run_time=0.5)

        self.play(ShowCreation(focus_rect))

        self.play(Write(linear_interpolation_tex),
                  Write(linear_interpolation_title))

        self.wait(2)

        def update_text_blue(obj):
            obj.next_to(focus_rect, UP)

        def update_text_red(obj):
            obj.next_to(focus_rect, UP * 3)

        blue_amount.add_updater(update_text_blue)
        red_amount.add_updater(update_text_red)
        self.add(blue_amount, red_amount)

        color_vecs = []

        self.wait(2)

        blue_amount_tex = TexMobject("\\frac{"+str(n-1)+"}{" +str(n)+"}").move_to(linear_interpolation_tex[0])
        red_amount_tex = TexMobject("\\frac{"+str(1) + "}{"+str(n)+"}").move_to(linear_interpolation_tex[4])
        # first colored pixel
        self.play(focus_rect.move_to, boxes[1],
                  ChangeDecimalToValue(blue_amount, 100 - (14.28571 * 1)),
                  ChangeDecimalToValue(red_amount, 0 + (14.28571 * 1)),
                  ReplacementTransform(linear_interpolation_tex[0], blue_amount_tex),
                  ReplacementTransform(linear_interpolation_tex[4], red_amount_tex))

        self.wait(4)

        new_color = self.interpolate_redblue(14.28571 * 1 / 100)
        new_color_tex = TexMobject(self.show_rgb_vector(new_color).get_tex_string()).move_to(linear_interpolation_tex[8])
        new_color_tex.set_color(new_color)

        self.play(boxes[1].set_fill, new_color, 1,
                  boxes[1].set_color, new_color,
                  ReplacementTransform(linear_interpolation_tex[8], new_color_tex))
        if 1 != n:
            color_vecs.append(self.show_rgb_vector(new_color).next_to(boxes[1], UP))

        self.wait()

        old_blue_amount_tex = blue_amount_tex
        old_red_amount_tex = red_amount_tex
        old_new_color_tex = new_color_tex


        for i in range(2, n + 1):

            blue_amount_tex = TexMobject("\\frac{" + str(n-i) + "}{" + str(n) + "}").move_to(old_blue_amount_tex)
            red_amount_tex = TexMobject("\\frac{" + str(i) + "}{" + str(n) + "}").move_to(old_red_amount_tex)

            self.play(focus_rect.move_to, boxes[i],
                      ChangeDecimalToValue(blue_amount, 100 - (14.28571 * i)),
                      ChangeDecimalToValue(red_amount, 0 + (14.28571 * i)),
                      ReplacementTransform(old_blue_amount_tex, blue_amount_tex),
                      ReplacementTransform(old_red_amount_tex, red_amount_tex))

            new_color = self.interpolate_redblue(14.28571 * i / 100 + 0.001)
            new_color_tex = TexMobject(self.show_rgb_vector(new_color).get_tex_string()).move_to(old_new_color_tex)
            new_color_tex.set_color(new_color)
            self.play(boxes[i].set_fill, new_color, 1,
                      boxes[i].set_color, new_color,
                      ReplacementTransform(old_new_color_tex, new_color_tex))
            if i != n:
                color_vecs.append(self.show_rgb_vector(new_color).next_to(boxes[i], UP))

            old_blue_amount_tex = blue_amount_tex
            old_red_amount_tex = red_amount_tex
            old_new_color_tex = new_color_tex

        self.play(FadeOutAndShiftDown(blue_amount),
                  FadeOutAndShiftDown(red_amount),
                  Uncreate(focus_rect),
                  FadeOut(old_new_color_tex),
                  FadeOut(old_red_amount_tex),
                  FadeOut(old_blue_amount_tex),
                  FadeOut(linear_interpolation_title),
                  *[FadeOut(linear_interpolation_tex[i]) for i in (1,2,3,5,6,7)])


        self.save_color_vec(color_vecs, vertex_save[7])

        self.play(FadeOut(interpolation_text))

    def save_color_vec(self,color_vecs, save_dest):
        self.play(AnimationGroup(*[FadeInFromDown(vec_color) for vec_color in color_vecs], lag_ratio=0.1))
        save_vertex_text = TextMobject(self.script["save"]).shift(UP)
        self.play(Write(save_vertex_text))

        self.wait()
        self.play(FadeOut(save_vertex_text))
        self.play(AnimationGroup(*[ReplacementTransform(vec, save_dest) for vec in color_vecs], lag_ratio=0.1))

        self.wait()


    def construct(self):
        self.define_interpolation()

    def interpolate_redblue(self, amount):
        rgb_inter = interpolate(hex_to_rgb(BLUE), hex_to_rgb(RED), amount)
        return rgb_to_hex(rgb_inter)

    def show_rgb_vector(self,color):
        rgb = hex_to_rgb(color)
        return TexMobject(r"\begin{bmatrix}" + str(int(rgb[0]*255)) +
                          r"\\" + str(int(rgb[1]*255)) +
                          r"\\" + str(int(rgb[2]*255)) + r"\end{bmatrix}")\
                        .scale(0.7).set_color(color)


class Rasterization(Scene):
    CONFIG = {
        "screen_pixels": {
            "color": WHITE,
            "amount": 144,  # 12 x 12
            "side_length": 0.5,
            "res_width": 12,
            "res_height": 12,
            "startpoint": DOWN * 2.75 + RIGHT * 0.5,
            "stroke_opacity": 0.5,
            "stroke_width": 0.8,

        },
        "line_pixels": {
            "color": WHITE,
            "amount": 8,
            "side_length": 1,
            "stroke_width": 2,
            "stroke_opacity": 1
        },
        "screen_kwargs": {
            "color": RED,
            "side_length": 6,
            "mark_paths_closed": True,
            "close_new_points": True,
            "center": RIGHT*3.25,
        },
        "vertices": (
            {
                "index": 128,
                "color": RED,
                "fill_opacity": 0.0,
                "position": (8, 10),
                "color_rgb": (255, 0, 0),#for vertex
            },
            {
                "index": 37,
                "color": BLUE,
                "fill_opacity": 0.0,
                "position": (1, 3),
                "color_rgb": (0, 102, 225),#for vertex
            },
            {
                "index": 21,
                "color": GREEN,
                "fill_opacity": 0.0,
                "position": (9, 1),
                "color_rgb": (0, 255, 0),#for vertex
            }

        ),

        "script": {
            "screen_pixel": "This is our Pixel Screen",
            "screen_def": "Resolution",
            "vertex_def": "Vertices",
            "raster_title": "Rasterization",
            "raster_exp": "Use the Bresenham \\\\ algorithm",
            "save_vertex": "Save each position \\\\ as its new vertex",
            "search_color": "which color?"
        }

    }
    def construct(self):

        pixel = PixelBox(self.screen_pixels, self.vertices)

        self.defineScreen(pixel)

        self.play(AnimationGroup(*[Write(number) for number in pixel.height], lag_ratio=0.1),
                  AnimationGroup(*[Write(number) for number in pixel.width], lag_ratio=0.1))

        vertex_save_text = self.defineVertices(pixel.boxes)

        self.define_raster(pixel)

        self.wait()
        box_line1 = []
        positions = []
        save_positions = TextMobject(self.script["save_vertex"]).to_edge(LEFT)
        box_lines = pixel.border_boxes
        for box_line, i in zip(box_lines, range(len(box_lines))):
            for box, ii in zip(box_line, range(len(box_line))):
                if i == 0:
                    box_line1.append(box.copy())
                if ii != 0 and ii != len(box_line)-1:
                    positions.append(Dot(box.get_center()))

        self.play(AnimationGroup(*[FadeIn(pos) for pos in positions], lag_ratio=0.1),
                  Write(save_positions))

        self.wait()

        self.play(*[FadeOut(pos) for pos in positions],
                  FadeIn(vertex_save_text[7]))
        self.play(FadeOut(save_positions))

        interpolation_title = self.define_search_color(box_line1[3])

        self.add(*[box for box in box_line1])
        box_line1.reverse()



        self.play(
                  *[FadeOut(number) for number in pixel.height],
                  *[FadeOut(number) for number in pixel.width],
                  *[FadeOut(box) for box in pixel.boxes]
                  )

        pixel_line = PixelLine(self.line_pixels)

        self.play(*[ReplacementTransform(originial, targed) for originial, targed in zip(box_line1, pixel_line.boxes)])

        self.wait(2)



    def define_raster(self, pixel):
        vertices = pixel.vertex_box

        border_line_text = TextMobject(self.script["raster_title"]).to_corner(UL).shift(RIGHT)
        raster_exp_text = TextMobject(self.script["raster_exp"]).to_edge(LEFT)

        focuses = self.generate_focus_cubes(vertices)

        apply_bresenham = TexMobject("Bresenham(", "x,y", ")").move_to(raster_exp_text).shift(UP*2)

        point_tex = self.place_3_to_start(apply_bresenham.copy().shift(DOWN), self.generate_color_tex())

        steigung_tex = self.generate_3_tex(apply_bresenham.copy().shift(DOWN),
                                           ("y =\\frac{7}{8} x + 2,125",
                                            "y = -\\frac{1}{4}x + 3,25",
                                            "y = -9x + 82"))


        lines = pixel.border_lines

        box_lines = pixel.border_boxes



        self.play(Write(border_line_text))

        self.play(Write(raster_exp_text))

        self.wait(8)

        self.play(FadeOut(raster_exp_text))

        self.play(Write(apply_bresenham))

        self.play(ShowCreation(focuses[0][0]), ShowCreation(focuses[0][1]))

        self.play(Write(point_tex[0]), run_time=2)
        self.wait(2)
        self.play(ReplacementTransform(point_tex[0], steigung_tex[0]))
        self.play(ShowCreation(lines[0]))
        self.wait(2)
        self.play(ReplacementTransform(steigung_tex[0], apply_bresenham[1]))
        for ii in range(1, len(box_lines[0]) - 1):
            box_lines[0][ii].set_stroke(YELLOW, 2, 1)

        self.play(FadeOut(lines[0]),
                  AnimationGroup(*[FadeIn(box_lines[0][ii]) for ii in range(1, len(box_lines[0]) - 1)]), lag_ratio=0.1)

        self.wait(2)

        for i in range(1,3):
            self.play(ReplacementTransform(focuses[i - 1][0], focuses[i][0]),
                          ReplacementTransform(focuses[i - 1][1], focuses[i][1]))
            self.play(Write(point_tex[i]))
            self.play(ReplacementTransform(point_tex[i], steigung_tex[i]))
            self.play(ShowCreation(lines[i]))
            self.play(ReplacementTransform(steigung_tex[i], apply_bresenham[1] ))
            for ii in range(1, len(box_lines[i])-1):
                box_lines[i][ii].set_stroke(YELLOW, 2, 1)

            self.play(FadeOut(lines[i]), AnimationGroup(*[FadeIn(box_lines[i][ii]) for ii in range(1, len(box_lines[i])-1)]), lag_ratio=0.1)

        self.play(FadeOutAndShiftDown(apply_bresenham), FadeOut(focuses[2][0]), FadeOut(focuses[2][1]))

        self.play(FadeOut(border_line_text))

    def define_search_color(self, search_box):
        search_color_box = Square(**self.screen_pixels).scale(2).to_edge(LEFT).shift(RIGHT*2).set_stroke(YELLOW, 2, 1)
        search_text = TextMobject(self.script["search_color"]).next_to(search_color_box,DOWN)
        question_mark = TextMobject("?").scale(2).move_to(search_color_box)

        interpolation_text = TextMobject("Interpolation").scale(1).move_to(search_color_box)\
                                                         .set_color_by_gradient(BLUE_A, RED)


        self.play(ReplacementTransform(search_box.copy(), search_color_box), run_time=2)
        self.play(Write(question_mark), Write(search_text))
        self.wait(3)
        self.play(ReplacementTransform(search_color_box, interpolation_text),
                  ReplacementTransform(search_text, interpolation_text),
                  ReplacementTransform(question_mark, interpolation_text))

        self.wait(2)
        self.play(ReplacementTransform(interpolation_text, interpolation_text.copy().to_corner(UL)))

        return interpolation_text

    def generate_3_tex(self, start_position, content):
        return self.place_3_to_start(start_position, (TexMobject(content[0]),
                                                      TexMobject(content[1]),
                                                      TexMobject(content[2])))

    def place_3_to_start(self, start_position, objects):
        tex1 = objects[0].next_to(start_position, BOTTOM)
        tex2 = objects[1].next_to(start_position, BOTTOM)
        tex3 = objects[2].next_to(start_position, BOTTOM)

        return tex1, tex2, tex3

    def generate_color_tex(self):
        t1 = TexMobject("p0(8,10)", ",", " p1(1,3)")
        t2 = TexMobject("p1(1,3)", ",", "  p2(9,1)")
        t3 = TexMobject("p2(9,1)", ",", "  p0(8,10)")

        t1[0].set_color(RED)
        t1[2].set_color(BLUE)
        t2[0].set_color(BLUE)
        t2[2].set_color(GREEN)
        t3[0].set_color(GREEN)
        t3[2].set_color(RED)

        return t1, t2, t3

    def generate_focus_cubes(self, pos):
        f1 = (SurroundingRectangle(pos[0], buff=0.1), SurroundingRectangle(pos[1], buff=0.1))
        f2 = (SurroundingRectangle(pos[1], buff=0.1), SurroundingRectangle(pos[2], buff=0.1))
        f3 = (SurroundingRectangle(pos[2], buff=0.1), SurroundingRectangle(pos[0], buff=0.1))

        return f1, f2, f3

    def defineVertices(self,pixel_boxes):

        vertex_save = TexMobject("[",  # 0
                                 "v0",  # 1
                                 ",",  # 2
                                 "v1",  # 3
                                 ",",  # 4
                                 "v2",  # 5
                                 ",",    # 6
                                 "...",  # 7
                                 "]"  # 8
                                 ).to_corner(UR).scale(0.5)
        position_vector = []
        color_vector = []

        vertex_def = TextMobject(self.script["vertex_def"])
        old_pos = vertex_def.to_corner(UL)
        self.play(Write(old_pos))
        pointer = Dot()

        newPixel = Square(**self.screen_pixels) \
            .move_to(pixel_boxes[self.vertices[0]["index"]]
                     .get_center())
        actual_pixel = pixel_boxes[self.vertices[0]["index"]]
        newPixel.set_color(self.vertices[0]["color"]).set_fill(self.vertices[0]["color"], 1)

        old_pointer = pointer.copy()
        pointer.move_to(actual_pixel.get_center())
        self.play(ReplacementTransform(old_pointer, pointer))

        position_vector.append(TexMobject("v" + str(0),
                                          r" = p\begin{bmatrix} " + str(self.vertices[0]["position"][0]) + r"  \\ " + str(
                                              self.vertices[0]["position"][1]) + r" \end{bmatrix}\\ ") \
                               .next_to(old_pos, DOWN * 3))

        color_vector.append(TexMobject(r" c\begin{bmatrix} " + str(self.vertices[0]["color_rgb"][0]) + r"  \\ " + str(
            self.vertices[0]["color_rgb"][1]) + r"  \\" + str(self.vertices[0]["color_rgb"][2]) + r"\end{bmatrix}\\ ") \
                            .next_to(position_vector[0]).set_color(self.vertices[0]["color"]))

        old_pos = position_vector[0]
        self.play(Write(position_vector[0]))
        actual_pixel.set_color(self.vertices[0]["color"]).set_fill(self.vertices[0]["color"], 1)
        self.play(FadeIn(actual_pixel))
        self.play(FadeOut(pointer))

        self.wait(2)
        self.play(ReplacementTransform(position_vector[0].copy(), color_vector[0]))
        self.wait(3)

        for vertex, i in zip(self.vertices, range(len(self.vertices))):
            if i == 0:
                continue

            newPixel = Square(**self.screen_pixels)\
                .move_to(pixel_boxes[vertex["index"]]
                         .get_center())
            actual_pixel = pixel_boxes[vertex["index"]]
            newPixel.set_color(vertex["color"]).set_fill(vertex["color"], 1)

            old_pointer = pointer.copy()
            pointer.move_to(actual_pixel.get_center())
            self.play(ReplacementTransform(old_pointer, pointer))

            position_vector.append(TexMobject("v" + str(i),
                                              r" = p\begin{bmatrix} " + str(vertex["position"][0]) + r"  \\ " + str(
                                                  vertex["position"][1]) + r" \end{bmatrix}\\ ") \
                                   .next_to(old_pos, DOWN*3))

            color_vector.append(TexMobject(r" c\begin{bmatrix} " + str(vertex["color_rgb"][0]) + r"  \\ " + str(
                vertex["color_rgb"][1]) + r"  \\" + str(vertex["color_rgb"][2]) + r"\end{bmatrix}\\ ") \
                                .next_to(position_vector[i]).set_color(vertex["color"]))

            old_pos = position_vector[i]
            self.play(Write(position_vector[i]),run_time=1.5)
            actual_pixel.set_color(vertex["color"]).set_fill(vertex["color"], 1)
            self.play(FadeIn(actual_pixel))
            self.play(FadeOut(pointer))

            self.play(ReplacementTransform(position_vector[i].copy(), color_vector[i]))


        self.wait(1)

        self.play(FadeOut(vertex_def),
                  *[FadeOut(position_vector[p][1]) for p in range(len(position_vector))],
                  *[FadeOut(color_vector[c]) for c in range(len(color_vector))])

        self.play(Write(vertex_save[8]),
                  *[Write(vertex_save[r]) for r in range(0, 8, 2)],
                  *[ReplacementTransform(position_vector[i][0], vertex_save[s])
                    for i, s in zip(range(len(self.vertices)), range(1, 7, 2))])

        self.wait(2)

        return vertex_save

    def defineScreen(self, pixel):
        simulateScreenRect = Square(**self.screen_kwargs)
        simulateScreenRect.move_to(self.screen_kwargs["center"])

        #animated the screen
        self.play(AnimationGroup(*[ShowCreation(pixelBox) for pixelBox in pixel.boxes], lag_ratio=0.01))

        screenTitle = TextMobject(self.script["screen_pixel"]).next_to(simulateScreenRect, UP)
        self.play(ShowCreation(simulateScreenRect), Write(screenTitle), run_time=1)

        self.wait(2)

        width_brace = Brace(simulateScreenRect, LEFT, buff=SMALL_BUFF)
        w_brace_text = width_brace.get_text("12px").scale(0.9)
        height_brace = Brace(simulateScreenRect, DOWN, buff=SMALL_BUFF)
        h_brace_text = height_brace.get_text("12px").scale(0.9)
        screen_res_text = TextMobject(self.script["screen_def"]).next_to(simulateScreenRect, UP)

        self.play(GrowFromCenter(width_brace),
                  GrowFromCenter(height_brace),
                  FadeIn(w_brace_text),
                  FadeIn(h_brace_text),
                  ReplacementTransform(screenTitle, screen_res_text))

        self.wait(3)

        self.play(ShrinkToCenter(width_brace),
                  ShrinkToCenter(height_brace),
                  FadeOut(w_brace_text),
                  FadeOut(h_brace_text),
                  FadeOut(screen_res_text),
                  Uncreate(simulateScreenRect))

    def devSetUp(self,pixel_boxes):
        vertex_save = TexMobject("[",  # 0
                                 "v0",  # 1
                                 ",",  # 2
                                 "v1",  # 3
                                 ",",  # 4
                                 "v2",  # 5
                                 ","    # 6
                                 "..."  # 7
                                 "]"  # 8
                                 ).to_corner(UR).scale(0.5)
        vertex_box = []
        for vertex in self.vertices:
            newPixel = Square(**self.screen_pixels).move_to(pixel_boxes[vertex["index"]].get_center())
            newPixel.set_color(vertex["color"]).set_fill(vertex["color"], 1)
            vertex_box.append(newPixel)
            self.add(newPixel)
        self.add(vertex_save)
        self.add(*[pixel for pixel in pixel_boxes])
        return vertex_box, vertex_save


class Intro(Scene):
    def construct(self):
        title = TextMobject("How to rasterize \\\\ a triangle")
        title.scale(2)

        self.play(Write(title), run_time=2)

        self.wait(1.5)

        #self.play(FadeOut(title))

class Outro(Scene):
    CONFIG = {
        "should_center": True,
        "height": 2,
        "width": None,
        # Must be filled in in a subclass, or when called
        "file_name": None,
        "unpack_groups": True,  # if False, creates a hierarchy of VGroups
        "stroke_width": 0,
        "fill_opacity": 0.0,
        # "fill_color" : LIGHT_GREY,
    }
    def construct(self):
        credit = TextMobject("This Video was made possible with:").to_edge(UP).shift(DOWN*0.5)
        manim_title = TextMobject("Manim").scale(2)
        sub_title = TextMobject("(Mathematical Animation Engine)").scale(0.5)
        sponserd_by = TextMobject("And sponsored by:")
        bfh_logo = ImageMobject("BFH").scale(1.5)

        creator = TextMobject("voice and animatinos by Cédric Girardin").scale(0.8)

        manim_title.next_to(credit, DOWN)
        sub_title.next_to(manim_title, DOWN)
        sponserd_by.next_to(sub_title,DOWN)
        bfh_logo.next_to(sponserd_by,DOWN)
        creator.next_to(bfh_logo, DOWN)

        self.play(Write(credit))
        self.play(Write(manim_title))
        self.play(Write(sub_title))

        self.play(Write(sponserd_by), )
        self.play(FadeIn(bfh_logo))

        self.play(Write(creator))

        self.wait()





class AllScenes(Intro, InterpolatedPoly, Rasterization, Interpolation, ScanLine, Outro):
    def construct(self):

        Intro.construct(self)
        self.clear()

        InterpolatedPoly.construct(self)
        self.clear()

        Rasterization.construct(self)
        self.clear()

        Interpolation.construct(self)
        self.clear()

        ScanLine.construct(self)

        self.clear()


        Outro.construct(self)
        self.clear()


