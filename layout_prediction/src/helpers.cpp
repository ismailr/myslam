#include <layout_prediction/helpers.h>

double calculate_slope (line_segment ls)
{
	if(ls.p.x == ls.q.x)
		return 0.0;

	// slope = (y2 - y1)/(x2 - x1)
	float dividend = (ls.q.y - ls.p.y);
	float divisor = (ls.q.x - ls.p.x);

	return dividend/divisor;
}

double calculate_slope (line_segment_stamped ls)
{
    line_segment _ls;
    _ls.p = ls.p.point;
    _ls.q = ls.q.point;

	return calculate_slope (_ls);
}

double calculate_slope (line l)
{
    line_segment _ls;
    _ls.p = l.p.point;
    _ls.q = l.q.point;

	return calculate_slope (_ls);
}

line_eq points_to_line_eq (line_segment ls)
{
	float slope = calculate_slope (ls);

	// Line eq. from two points
	// (y2-y1) = m(x2-x1)
	// y2 = m(x2-x1) + y1
	// y2 = m.x2 - m.x1 + y1
	// y2 = m.x2 + c
	// with c = m.x1 + y1
	
	float intercept = slope * ls.p.x + ls.p.y;

	line_eq calculated_line;
    calculated_line.slope = slope;
	calculated_line.intercept = intercept;
	       
	return calculated_line;
}

line_eq points_to_line_eq (line_segment_stamped ls)
{
    line_segment _ls;
    _ls.p = ls.p.point;
    _ls.q = ls.q.point;
	       
	return points_to_line_eq (_ls);
}

