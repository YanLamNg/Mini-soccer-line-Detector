import cv2 as cv

from base.base_module import BaseModule
from base.configuration import Configuration
from field_objects import Field, Lines

from line_segment_detector_mat import LineSegmentDetectorMat


class LineSegmentDetectorModule(Configuration):
    def __init__(self, base_module, configuration_directory, event, robot_size):
        super(LineSegmentDetectorModule, self).__init__(
            configuration_directory=configuration_directory, event=event, module='line_segment_detector', robot_size=robot_size)

        self.field = Field(configuration=self.config['field'], module=self)
        self.lines = Lines(configuration=self.config['lines'], module=self)

        self.base = base_module
        self.base.add_debug_objects([self.field, self.lines])

        self.colourspace_roi = None

    @staticmethod
    def is_enabled(base):
        return 'line_segment_detector' in base.config['modules']

    def update(self):
        self.config['field'] = self.field.export_configuration()
        self.config['lines'] = self.lines.export_configuration()
        self.save()



    def track_lines(self, input_mat, output_mat, mask=False):
        # Create LSD mat
        lsd_mat = LineSegmentDetectorMat(frame=input_mat.frame)
        self.field.mat = lsd_mat
        self.lines.mat = lsd_mat

        # Extract background from soccer field
        lsd_mat.background_mask(thresh_lb=self.field.lower_bound, thresh_ub=self.field.upper_bound,
                                min_area=self.field.min_area, line_width=self.lines.max_width)

        # Mask out parts of output image that is not the field
        if mask:
            output_mat.frame = cv.bitwise_and(
                output_mat.frame, output_mat.frame, mask=lsd_mat.frame)

        # Extract lines from soccer field and detect lines using LSD
        lsd_mat.extract_lines(thresh_lb=self.lines.lower_bound,
                              thresh_ub=self.lines.upper_bound)

        # Find the lines and corners, classify each line
        lsd_mat.lsd(max_distance_apart=self.lines.max_distance_apart,
                    min_length=self.lines.min_length)
        lsd_mat.find_corners(
            max_distance_apart=self.lines.corner_max_distance_apart)
        field_lines = lsd_mat.classify_lines()

        # Draw lines if found
        if field_lines:
            for line_type in field_lines.keys():
                if line_type == 'boundary':
                    colour = self.lines.output_boundary_colour
                elif line_type == 'goal_area':
                    colour = self.lines.output_goal_area_colour
                elif line_type == 'center':
                    colour = self.lines.output_center_colour
                else:
                    colour = self.lines.output_undefined_colour

                for line_set in field_lines[line_type]:
                    for line in line_set[0]:
                        pt1, pt2 = line.to_cv_line()
                        cv.line(output_mat.frame, pt1=pt1, pt2=pt2,
                                color=colour, thickness=3)

        # Publish lines found
        self.lines.publish_msg(field_lines if field_lines else [])

        return output_mat
