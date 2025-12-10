module.exports = {
  docs: [
    {
      type: 'doc',
      id: 'intro', 
    },
    {
      type: 'doc',
      id: 'why-physical-ai-matters',
    },
    {
      type: 'doc',
      id: 'learning-outcomes',
    },
    {
      type: 'doc',
      id: 'hardware-requirements',
    },
    {
      type: 'category',
      label: 'Course Modules',
      items: [
        'course-modules/index',
        {
          type: 'category',
          label: 'Module 1',
          items: [
            'course-modules/week-3/index',
            'course-modules/week-4/index',
            'course-modules/week-5/index',
          ],
        },
        {
          type: 'category',
          label: 'Module 2',
          items: [
            'course-modules/week-6/index',
            'course-modules/week-7/index',
          ],
        },
        {
          type: 'category',
          label: 'Module 3',
          items: [
            'course-modules/week-8/index',
            'course-modules/week-9/index',
            'course-modules/week-10/index',
          ],
        },
        {
          type: 'category',
          label: 'Module 4',
          items: [
            'course-modules/week-1/index',
            'course-modules/week-11/index',
            'course-modules/week-12/index',
            'course-modules/week-13/index',
          ],
        },
      ],
    },
  ],
};